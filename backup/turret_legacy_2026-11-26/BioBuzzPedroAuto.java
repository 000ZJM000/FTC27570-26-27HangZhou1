package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotStateStorage;
import org.firstinspires.ftc.teamcode.biobuzz.core.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.core.ControlLoop;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.localization.PedroPinpointLocalizer;
import org.firstinspires.ftc.teamcode.biobuzz.localization.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming.AutoVolley;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming.CornerAim;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming.FieldTargets;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming.ShotMotion;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming.ShotSequence;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotMap;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake.TaskServos;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;
import org.json.JSONException;

import java.io.IOException;
import java.util.function.Supplier;

@Autonomous(name = "BIOBUZZ Pedro 吸两组设三组2T", group = "BIOBUZZ")
public class BioBuzzPedroAuto extends LinearOpMode {
    enum AutoMode { TWO_GROUP_9_LEG, WAIT_SHOOT_PARK }

    private static final double PINPOINT_PAUSE_SECONDS = 0.25;
    private static final long TIMEOUT_BRAKE_MS = 100;
    private static final long FINAL_SHOOT_WINDOW_MS = 2000L;

    private enum LegResult { REACHED, TIMED_OUT, STOPPED }

    private final AutoMode autoMode;

    public BioBuzzPedroAuto() {
        this(AutoMode.TWO_GROUP_9_LEG);
    }

    protected BioBuzzPedroAuto(AutoMode autoMode) {
        this.autoMode = autoMode;
    }

    @Override public void runOpMode() throws InterruptedException {
        RobotStateStorage.clearAutoPose();
        BatteryVoltage battery = new BatteryVoltage(hardwareMap);
        PinpointPose pinpoint = new PinpointPose(hardwareMap, battery);
        PedroDrive mecanum = null;
        Mechanisms mechanisms = null;
        String status;
        try (ControlLoop loop = new ControlLoop(hardwareMap)) {
            mechanisms = new Mechanisms(hardwareMap, telemetry, battery, pinpoint);
            PedroRoute route = PedroRoute.load(hardwareMap.appContext.getAssets(),
                    autoMode == AutoMode.WAIT_SHOOT_PARK ? PedroRoute.WAIT_SHOOT_PARK_ASSET : PedroRoute.ASSET);
            if (autoMode == AutoMode.TWO_GROUP_9_LEG && route.legs.size() != 9) {
                throw new JSONException("机构流程需要 9 段 PP 路径");
            }
            if (autoMode == AutoMode.WAIT_SHOOT_PARK && route.legs.size() < 2) {
                throw new JSONException("等待打预制 Park 流程至少需要 2 段 PP 路径");
            }
            mecanum = new PedroDrive(hardwareMap);
            mecanum.stop(true);
            PedroPinpointLocalizer localizer = new PedroPinpointLocalizer(pinpoint);
            Follower follower = new Follower(localizer, mecanum, new Foresight(foresightConfig()));

            boolean started = false;
            boolean initAnchored = false;
            double nextInit = 0;
            Pose firstEnd = route.legs.get(0).path.endPose();
            Pose end = route.legs.get(route.legs.size() - 1).path.endPose();
            while (opModeInInit()) {
                double now = loop.beginFrame();
                if (!started && now >= nextInit) {
                    started = pinpoint.startImu();
                    nextInit = now + 1;
                }
                localizer.sample();
                if (!initAnchored) initAnchored = PedroPinpointLocalizer.anchorRouteStart(pinpoint, route.start);
                telemetry.addData("Pinpoint", pinpoint.status);
                telemetry.addData("Hub IMU 备援", pinpoint.hubImuSummary());
                telemetry.addData("FR / FL", mechanisms.frontStatus());
                telemetry.addData("射击数据", mechanisms.shotMapStatus());
                telemetry.addData("起点 X / Y / 车头", "%.2f / %.2f in / %.0f°",
                        route.start.x(), route.start.y(), Math.toDegrees(route.start.heading()));
                if (autoMode == AutoMode.WAIT_SHOOT_PARK) {
                    long totalWaitMs = Math.max(0L, route.legs.get(0).waitBefore + route.legs.get(0).waitAfter);
                    long preWaitMs = Math.max(0L, totalWaitMs - FINAL_SHOOT_WINDOW_MS);
                    telemetry.addData("等待 / 射击", "总等待 %.1f s（静候 %.1f s + 最后 %.1f s 射球后 Park）",
                            totalWaitMs / 1000.0, preWaitMs / 1000.0, FINAL_SHOOT_WINDOW_MS / 1000.0);
                }
                telemetry.addData("第 1 段末端车头", "%.0f°", Math.toDegrees(firstEnd.heading()));
                telemetry.addData("终点 X / Y / 车头", "%.2f / %.2f in / %.0f°",
                        end.x(), end.y(), Math.toDegrees(end.heading()));
                telemetry.addLine("PP 场地图角度 | 0° 朝右 · 90° 朝上 · 180° 朝左 · 270° 朝下");
                telemetry.addLine("保持底盘静止，把云台轻摆到最右端后 START");
                telemetry.update();
                sleep(25);
            }
            if (isStopRequested()) return;

            double settleDeadline = Maths.time() + 0.6;
            while (!initAnchored && opModeIsActive() && Maths.time() < settleDeadline) {
                loop.beginFrame();
                localizer.sample();
                initAnchored = PedroPinpointLocalizer.anchorRouteStart(pinpoint, route.start);
                if (!initAnchored) sleep(10);
            }
            if (!initAnchored || !pinpoint.positionValid) {
                throw new IllegalStateException("Pinpoint 定位未就绪：" + pinpoint.status);
            }
            mechanisms.start();
            recordState(pinpoint, mechanisms);
            status = autoMode == AutoMode.WAIT_SHOOT_PARK
                    ? runWaitShootPark(route, follower, mecanum, localizer, loop, pinpoint, mechanisms)
                    : runTwoGroupNineLeg(route, follower, mecanum, localizer, loop, pinpoint, mechanisms);
            if (status == null) return;
        } catch (IOException | JSONException | RuntimeException failure) {
            status = "自动停止：" + failure.getMessage();
        } finally {
            final PedroDrive driveToStop = mecanum;
            if (driveToStop != null) ControlLoop.safeStop("Pedro 底盘", () -> driveToStop.stop(true));
            if (mechanisms != null) mechanisms.stop();
            recordState(pinpoint, mechanisms);
        }

        while (!isStopRequested()) {
            pinpoint.update();
            if (pinpoint.hasPositionAnchor()) {
                RobotStateStorage.recordAutoPose(pinpoint.forward, pinpoint.left, pinpoint.heading);
                telemetry.addData("Pinpoint 右 / 前 / 车头", "%.2f / %.2f 格 / %+.1f°",
                        -pinpoint.left / RobotConfig.TILE_MM, pinpoint.forward / RobotConfig.TILE_MM,
                        Math.toDegrees(pinpoint.heading));
            }
            telemetry.addData("Pinpoint", pinpoint.status);
            if (pinpoint.usingHubImu || pinpoint.hubFallbackEvents > 0 || pinpoint.hubImuMismatch()) {
                telemetry.addData("Hub IMU 备援", pinpoint.hubImuSummary());
            }
            telemetry.addData("自动接续", RobotStateStorage.hasAutoPose() && RobotStateStorage.hasTurretHome()
                    ? "已保存位姿与云台零位，可直接进手动" : "接续数据不完整");
            telemetry.addLine(status);
            telemetry.update();
            sleep(50);
        }
    }

    private String runTwoGroupNineLeg(PedroRoute route, Follower follower, PedroDrive mecanum,
                                      PedroPinpointLocalizer localizer, ControlLoop loop,
                                      PinpointPose pinpoint, Mechanisms mechanisms) throws InterruptedException {
        int timedOut = 0;
        boolean rejoin = false;
        for (int index = 0; index < route.legs.size(); index++) {
            PedroRoute.Leg leg = route.legs.get(index);
            int number = index + 1;
            if (number == 2 || number == 6) mechanisms.collect(true);
            if (number == 6) mechanisms.forceLowerFront(true);
            if (!waitAtStop(leg.waitBefore, localizer, loop, pinpoint, mechanisms)) return null;
            String name = String.format("%d/%d %s", number, route.legs.size(), leg.name);
            LegResult result = followLeg(follower, mecanum, pinpoint, leg, name, rejoin, number == 9, loop, mechanisms);
            if (result == LegResult.STOPPED) return null;
            if (number == 5) mechanisms.forceLowerFront(true);
            if (number == 7 || number == 9) mechanisms.forceLowerFront(false);
            if (number == 8) mechanisms.collect(false);
            boolean hasPush = number == 6;
            rejoin = result == LegResult.TIMED_OUT;
            if (rejoin) {
                timedOut++;
                Pose target = leg.path.endPose();
                Pose measured = follower.pose();
                telemetry.log().add("%s 未到位，持续低速，超时跳过底盘。偏差 %.2f in / %.1f°", name,
                        measured.distance(target), Math.toDegrees(Maths.wrap(target.heading() - measured.heading())));
                telemetry.update();
                if (!hasPush && !waitAtStop(TIMEOUT_BRAKE_MS, localizer, loop, pinpoint, mechanisms)) return null;
            }
            if (hasPush) {
                mechanisms.collect(true);
                int cycles = Math.max(0, RobotConfig.AUTO_LEG6_BUMP_CYCLES);
                for (int cycle = 1; cycle <= cycles; cycle++) {
                    String cycleLabel = String.format("%s (%d/%d)", name, cycle, cycles);
                    if (!driveForwardOpenLoop(RobotConfig.AUTO_LEG6_BACK_SECONDS,
                            -Math.abs(RobotConfig.AUTO_LEG6_BUMP_POWER),
                            cycleLabel, mecanum, localizer, loop, pinpoint, mechanisms)) return null;
                    if (!driveForwardOpenLoop(RobotConfig.AUTO_LEG6_PUSH_SECONDS,
                            Math.abs(RobotConfig.AUTO_LEG6_BUMP_POWER),
                            cycleLabel, mecanum, localizer, loop, pinpoint, mechanisms)) return null;
                }
                rejoin = true;
            }
            if (number == 1 || number == 4 || number == 8) {
                if (number == 4) {
                    long delayMs = Math.round(Math.max(0.0, RobotConfig.AUTO_LEG4_SHOOT_DELAY_SECONDS) * 1000.0);
                    if (delayMs > 0 && !waitAtStop(delayMs, localizer, loop, pinpoint, mechanisms)) return null;
                }
                if (!shootAtStop(leg.waitAfter, name, localizer, loop, pinpoint, mechanisms)) return null;
            } else if (number == 2 || number == 5 || result == LegResult.REACHED || hasPush) {
                long waitMs = leg.waitAfter;
                if (number == 5) {
                    mechanisms.forceLowerFront(true);
                    long lowerSettleMs = Math.round(Math.max(0.0, RobotConfig.AUTO_FRONT_LOWER_SETTLE_SECONDS) * 1000.0);
                    waitMs = Math.max(waitMs, lowerSettleMs);
                    telemetry.log().add("路径 5 结束：放下 FR/FL（%s）", mechanisms.frontStatus());
                    telemetry.update();
                }
                if (waitMs > 0 && !waitAtStop(waitMs, localizer, loop, pinpoint, mechanisms)) return null;
            }
            if (number == 2) mechanisms.collect(false);
            if (number == 8) mechanisms.stopShooting();
        }
        return timedOut == 0 ? "路径完成" : String.format("路线执行结束，超时跳过 %d 段", timedOut);
    }

    private String runWaitShootPark(PedroRoute route, Follower follower, PedroDrive mecanum,
                                    PedroPinpointLocalizer localizer, ControlLoop loop,
                                    PinpointPose pinpoint, Mechanisms mechanisms) throws InterruptedException {
        int timedOut = 0;
        PedroRoute.Leg firstLeg = route.legs.get(0);
        String firstName = String.format("1/%d %s", route.legs.size(), firstLeg.name);
        LegResult firstResult = followLeg(follower, mecanum, pinpoint, firstLeg, firstName, false, false, loop, mechanisms);
        if (firstResult == LegResult.STOPPED) return null;
        boolean rejoin = firstResult == LegResult.TIMED_OUT;
        if (rejoin) {
            timedOut++;
            Pose target = firstLeg.path.endPose();
            Pose measured = follower.pose();
            telemetry.log().add("%s 未到位，持续低速，超时跳过底盘。偏差 %.2f in / %.1f°", firstName,
                    measured.distance(target), Math.toDegrees(Maths.wrap(target.heading() - measured.heading())));
            telemetry.update();
        }
        long totalWaitMs = Math.max(0L, firstLeg.waitBefore + firstLeg.waitAfter);
        long preWaitMs = Math.max(0L, totalWaitMs - FINAL_SHOOT_WINDOW_MS);
        if (!waitAtStop(preWaitMs, "等待发射倒计时", localizer, loop, pinpoint, mechanisms)) return null;
        if (!shootAtStop(FINAL_SHOOT_WINDOW_MS, "最后 2s 射出预制球", localizer, loop, pinpoint, mechanisms)) return null;
        mechanisms.stopShooting();

        for (int index = 1; index < route.legs.size(); index++) {
            PedroRoute.Leg leg = route.legs.get(index);
            int number = index + 1;
            if (!waitAtStop(leg.waitBefore, localizer, loop, pinpoint, mechanisms)) return null;
            String name = String.format("%d/%d %s", number, route.legs.size(), leg.name);
            boolean park = index == route.legs.size() - 1;
            LegResult result = followLeg(follower, mecanum, pinpoint, leg, name, rejoin, park, loop, mechanisms);
            if (result == LegResult.STOPPED) return null;
            rejoin = result == LegResult.TIMED_OUT;
            if (rejoin) {
                timedOut++;
                Pose target = leg.path.endPose();
                Pose measured = follower.pose();
                telemetry.log().add("%s 未到位，持续低速，超时跳过底盘。偏差 %.2f in / %.1f°", name,
                        measured.distance(target), Math.toDegrees(Maths.wrap(target.heading() - measured.heading())));
                telemetry.update();
            }
            if (leg.waitAfter > 0 && !waitAtStop(leg.waitAfter, localizer, loop, pinpoint, mechanisms)) return null;
        }
        return timedOut == 0 ? "路径完成" : String.format("路线执行结束，超时跳过 %d 段", timedOut);
    }

    private static void recordState(PinpointPose pinpoint, Mechanisms mechanisms) {
        if (pinpoint != null && pinpoint.hasPositionAnchor()) {
            RobotStateStorage.recordAutoPose(pinpoint.forward, pinpoint.left, pinpoint.heading);
        }
        if (mechanisms != null && mechanisms.hasTurretHome()) {
            RobotStateStorage.recordTurretHome(mechanisms.turretHomeTick());
            RobotStateStorage.recordTurretSample(mechanisms.turretLastTick(), mechanisms.turretAngle());
        }
    }

    private boolean waitAtStop(long millis, PedroPinpointLocalizer localizer, ControlLoop loop,
                               PinpointPose pinpoint, Mechanisms mechanisms) throws InterruptedException {
        return waitAtStop(millis, null, localizer, loop, pinpoint, mechanisms);
    }

    private boolean waitAtStop(long millis, String label, PedroPinpointLocalizer localizer, ControlLoop loop,
                               PinpointPose pinpoint, Mechanisms mechanisms) throws InterruptedException {
        if (millis <= 0) return opModeIsActive();
        return holdAtStop(millis, label, null, localizer, loop, pinpoint, mechanisms);
    }

    private boolean driveForwardOpenLoop(double seconds, double power, String name, PedroDrive drive,
                                         PedroPinpointLocalizer localizer, ControlLoop loop,
                                         PinpointPose pinpoint, Mechanisms mechanisms) {
        if (seconds <= 0) return opModeIsActive();
        double clampedPower = Maths.clip(power, -1.0, 1.0);
        double started = Maths.time();
        double until = started + seconds;
        double nextDisplay = started;
        DrivePowers forward = new DrivePowers(clampedPower, 0, 0);
        try {
            while (opModeIsActive()) {
                double frame = loop.beginFrame();
                if (frame >= until) break;
                localizer.sample();
                recordState(pinpoint, mechanisms);
                if (isStopRequested()) return false;
                drive.applyDrive(forward);
                mechanisms.update(frame, false, 0, false);
                if (frame >= nextDisplay) {
                    nextDisplay = frame + 0.12;
                    telemetry.addData(clampedPower < 0 ? "开环回退吸取" : "开环前进吸取",
                            "%s (%.2f / %.2f s, power %.2f)",
                            name, frame - started, seconds, clampedPower);
                    telemetry.addData("Pinpoint 右 / 前 / 车头", "%.2f / %.2f 格 / %+.1f°",
                            -pinpoint.left / RobotConfig.TILE_MM, pinpoint.forward / RobotConfig.TILE_MM,
                            Math.toDegrees(pinpoint.heading));
                    telemetry.update();
                }
            }
            return opModeIsActive();
        } finally {
            drive.stop(true);
        }
    }

    private boolean shootAtStop(long millis, String name, PedroPinpointLocalizer localizer,
                                ControlLoop loop, PinpointPose pinpoint, Mechanisms mechanisms)
            throws InterruptedException {
        AutoVolley volley = new AutoVolley(Maths.time(), millis / 1000.0);
        boolean active;
        try {
            active = holdAtStop(millis, name, volley, localizer, loop, pinpoint, mechanisms);
        } finally {
            mechanisms.stopFeeding();
        }
        if (active) telemetry.log().add("%s 供弹 %.2f 秒%s", name, volley.feedSeconds(),
                volley.finished() ? "" : "，射击等待超时");
        return active;
    }

    private boolean holdAtStop(long millis, String name, AutoVolley volley,
                               PedroPinpointLocalizer localizer, ControlLoop loop,
                               PinpointPose pinpoint, Mechanisms mechanisms) throws InterruptedException {
        double started = Maths.time();
        double totalSec = millis / 1000.0;
        double until = started + totalSec;
        double nextDisplay = started;
        while (opModeIsActive()) {
            double frame = loop.beginFrame();
            boolean triggerPressed = volley != null && volley.requesting(frame);
            if (frame >= until && !triggerPressed) break;
            localizer.sample();
            recordState(pinpoint, mechanisms);
            if (isStopRequested()) return false;
            boolean feeding = mechanisms.update(frame, true, 0, triggerPressed);
            if (volley != null) {
                volley.recordOutput(feeding);
                if (frame >= nextDisplay) {
                    nextDisplay = frame + 0.10;
                    mechanisms.showVolley(name, volley);
                    telemetry.update();
                }
            } else if (name != null && frame >= nextDisplay) {
                nextDisplay = frame + 0.12;
                double remaining = Math.max(0.0, until - frame);
                telemetry.addData(name, "剩余 %.1f / %.1f 秒", remaining, totalSec);
                mechanisms.showStandby();
                telemetry.update();
            }
        }
        return opModeIsActive();
    }

    private LegResult followLeg(Follower follower, PedroDrive drive, PinpointPose pinpoint, PedroRoute.Leg leg,
                                String name, boolean rejoin, boolean park, ControlLoop loop,
                                Mechanisms mechanisms) {
        Pose target = leg.path.endPose();
        double posTol = RobotConfig.AUTO_POS_TOLERANCE_IN;
        double headTol = Math.toRadians(RobotConfig.AUTO_HEADING_TOLERANCE_DEG);
        double stallSec = RobotConfig.AUTO_STALL_SECONDS;
        double stallSpeed = RobotConfig.AUTO_STALL_SPEED_IPS;
        double stallYaw = Math.toRadians(RobotConfig.AUTO_STALL_YAW_DEG_S);
        try {
            double stalledSince = Double.NaN;
            double invalidSince = Double.NaN;
            double nextDisplay = 0;
            boolean following = false;
            while (opModeIsActive()) {
                double frame = loop.beginFrame();
                try {
                    if (!following) {
                        follower.localizer.update();
                        Pose measured = follower.pose();
                        if (rejoin && measured.distance(target) <= posTol) {
                            follower.hold(target);
                        } else {
                            Path path = rejoin ? leg.fromMeasuredPose(measured) : leg.path;
                            follower.follow(path);
                        }
                        following = true;
                    } else {
                        follower.update();
                    }
                    recordState(pinpoint, mechanisms);
                    if (isStopRequested()) return LegResult.STOPPED;
                    invalidSince = Double.NaN;
                } catch (PedroPinpointLocalizer.PositionUnavailable failure) {
                    follower.drivetrain.stop();
                    stalledSince = Double.NaN;
                    if (!Double.isFinite(invalidSince)) invalidSince = frame;
                    if (frame - invalidSince >= PINPOINT_PAUSE_SECONDS) throw failure;
                    if (isStopRequested()) return LegResult.STOPPED;
                    mechanisms.update(frame, true, 0, false);
                    if (frame >= nextDisplay) {
                        nextDisplay = frame + 0.12;
                        telemetry.addData("路径", name);
                        telemetry.addData("Pinpoint", "暂停，等待有效定位");
                        telemetry.update();
                    }
                    continue;
                }
                mechanisms.update(frame, !drive.translating, drive.turn, false);
                if (park && pastMidpoint(follower)) mechanisms.lowerFront(false);
                double distance = follower.pose().distance(target);
                double turn = Math.abs(Maths.wrap(target.heading() - follower.pose().heading()));
                double speed = follower.velocity().toVector2D().magnitude();
                double yawRate = Math.abs(follower.velocity().omega);
                Path activePath = follower.currentPath();
                boolean nearPathEnd = follower.holding() || (activePath != null
                        && activePath.curve.remainingDistance(follower.parametricCompletion()) <= posTol);
                if (nearPathEnd && distance <= posTol && turn <= headTol
                        && speed <= 4 && yawRate <= Math.toRadians(20)) return LegResult.REACHED;
                if (speed < stallSpeed && yawRate < stallYaw) {
                    if (!Double.isFinite(stalledSince)) stalledSince = frame;
                    if (frame - stalledSince >= stallSec) return LegResult.TIMED_OUT;
                } else {
                    stalledSince = Double.NaN;
                }
                if (frame >= nextDisplay) {
                    nextDisplay = frame + 0.12;
                    telemetry.addData("路径", name);
                    if (Double.isFinite(stalledSince)) {
                        telemetry.addData("未到位，低速计时", "%.1f / %.1f 秒", frame - stalledSince, stallSec);
                    }
                    telemetry.addData("终点误差", "%.1f in / %.0f°", distance, Math.toDegrees(turn));
                    telemetry.addData("底盘实测速度", "%.2f in/s / %.1f°/s", speed, Math.toDegrees(yawRate));
                    telemetry.update();
                }
            }
            return LegResult.STOPPED;
        } finally {
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    private static boolean pastMidpoint(Follower follower) {
        if (follower.holding()) return true;
        Path path = follower.currentPath();
        return path != null && path.curve.remainingDistance(follower.parametricCompletion()) <= 0.5 * path.curve.length();
    }

    private static final class PedroDrive extends Mecanum {
        private boolean translating;
        private double turn;

        PedroDrive(HardwareMap map) {
            super(map, mecanumConfig());
        }

        @Override public void applyDrive(DrivePowers powers) {
            super.applyDrive(powers);
            translating = powers.forward() != 0 || powers.strafe() != 0;
            turn = Drive.turnFromWheels(wheelPowers[0], wheelPowers[2], wheelPowers[1], wheelPowers[3]);
        }

        @Override public void stop(boolean brake) {
            super.stop(brake);
            translating = false;
            turn = 0;
        }
    }

    private static MecanumConfig mecanumConfig() {
        return new MecanumConfig(c -> {
            c.frontLeftName.set(RobotConfig.DRIVE[0]);
            c.backLeftName.set(RobotConfig.DRIVE[1]);
            c.frontRightName.set(RobotConfig.DRIVE[2]);
            c.backRightName.set(RobotConfig.DRIVE[3]);
            c.frontLeftDirection.set(RobotConfig.DRIVE_DIRECTIONS[0]);
            c.backLeftDirection.set(RobotConfig.DRIVE_DIRECTIONS[1]);
            c.frontRightDirection.set(RobotConfig.DRIVE_DIRECTIONS[2]);
            c.backRightDirection.set(RobotConfig.DRIVE_DIRECTIONS[3]);
        });
    }

    private static ForesightConfig foresightConfig() {
        double forwardDecel = RobotConfig.BRAKE_FORWARD_MM_S2 / PedroPinpointLocalizer.MM_PER_INCH;
        double strafeDecel = RobotConfig.BRAKE_SIDE_MM_S2 / PedroPinpointLocalizer.MM_PER_INCH;
        return new ForesightConfig(c -> {
            c.forwardTranslational.set(Controller.proportional(RobotConfig.PEDRO_FWD_P));
            c.strafeTranslational.set(Controller.proportional(RobotConfig.PEDRO_STRAFE_P));
            c.headingFeedback.set(Controller.proportional(RobotConfig.PEDRO_HEADING_P));
            c.coast.set(Controller.sum(Controller.staticFeedforward(RobotConfig.PEDRO_COAST_S),
                    Controller.proportional(RobotConfig.PEDRO_COAST_P)));
            c.brake.set(Controller.proportionalFeedforward(RobotConfig.PEDRO_BRAKE_P));
            c.linearBrakeCoefficients.set(Matrix.diag(0.04, 0.04));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.5 / forwardDecel, 0.5 / strafeDecel));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.045, 0.005));
            c.maxAchievableForwardVelocity.set(RobotConfig.PEDRO_MAX_FWD_VEL);
            c.maxAchievableStrafeVelocity.set(RobotConfig.PEDRO_MAX_STRAFE_VEL);
            c.naturalForwardDeceleration.set(forwardDecel);
            c.naturalStrafeDeceleration.set(strafeDecel);
            c.maxPathSpeed.set(1.0);
            c.maxDecelerationConstraint.set(RobotConfig.PEDRO_MAX_DECEL > 0
                    ? RobotConfig.PEDRO_MAX_DECEL : ForesightConfig.Constraint.NONE);
            c.maxBrakingPower.set(Maths.clip(RobotConfig.PEDRO_MAX_BRAKE_POWER, 0.10, 1.0));
            c.headingDeviationTolerance.set(Math.toRadians(Math.max(5.0, RobotConfig.PEDRO_HEADING_DEV_TOL_DEG)));
            c.translationalDeviationTolerance.set(Math.max(1.0, RobotConfig.PEDRO_TRANS_DEV_TOL_IN));
            c.holdPointTranslationalScaling.set(Maths.clip(RobotConfig.PEDRO_HOLD_TRANS_SCALE, 0.15, 1.0));
            c.holdPointHeadingScaling.set(Maths.clip(RobotConfig.PEDRO_HOLD_HEADING_SCALE, 0.15, 1.0));
            c.headingDriveRatio.set(Maths.clip(RobotConfig.PEDRO_HEADING_DRIVE_RATIO, 0.10, 0.90));
            c.pathSkip.set(false);
        });
    }

    private static final class Mechanisms {
        private final Telemetry telemetry;
        private final PinpointPose pose;
        private final TaskServos front;
        private final Intake intake;
        private final Flywheel flywheel;
        private final Turret turret;
        private final ShotMap shotMap;
        private final ShotSequence shot;
        private final double maxRpm;
        private boolean armed, collecting, feeding;
        private double lastUpdate, startedAt = Double.NaN;
        private CornerAim aim;
        private FieldTargets fallbackAim;
        private String shotStatus = "等待 START";

        Mechanisms(HardwareMap map, Telemetry telemetry, BatteryVoltage battery, PinpointPose pose) {
            this.telemetry = telemetry;
            this.pose = pose;
            CalibrationStore store = new CalibrationStore(map.appContext);
            front = optional("FR/FL", () -> new TaskServos(map));
            intake = optional("吸取", () -> new Intake(map));
            flywheel = optional("飞轮", () -> new Flywheel(map, store, battery));
            turret = optional("云台", () -> new Turret(map, store, battery));
            shotMap = optional("射界数据", () -> new ShotMap(store));
            shot = new ShotSequence(turret, flywheel, intake, true);
            maxRpm = flywheel == null ? 0 : flywheel.maxTargetRpm();
            if (intake == null) {
                initializeServo(map, RobotConfig.INTAKE_LEFT_SERVO, RobotConfig.SL_UP);
                initializeServo(map, RobotConfig.INTAKE_RIGHT_SERVO, RobotConfig.SR_UP);
            }
            if (flywheel == null) initializeServo(map, RobotConfig.BLOCKER_SERVO, RobotConfig.SB_IDLE);
            initializeServo(map, RobotConfig.PITCH_SERVO, RobotConfig.ST_HOME);
            if (turret != null) turret.teachingMode(true);
        }

        void start() {
            if (front != null) front.moveToHome();
            if (turret != null) {
                turret.teachingMode(false);
                if (turret.calibrated()) {
                    turret.homeAtRight();
                    if (turret.isHomed()) {
                        RobotStateStorage.recordTurretHome(turret.homeTick());
                        RobotStateStorage.recordTurretSample(turret.lastTick(), turret.angle);
                    } else {
                        RobotStateStorage.clearTurretHome();
                    }
                }
            }
            armed = true;
            startedAt = lastUpdate = Maths.time();
        }

        boolean hasTurretHome() { return turret != null && turret.isHomed() && !turret.outOfTravel(); }
        int turretHomeTick() { return turret == null ? 0 : turret.homeTick(); }
        int turretLastTick() { return turret == null ? 0 : turret.lastTick(); }
        double turretAngle() { return turret == null ? 0 : turret.angle; }
        String frontStatus() { return front == null ? "初始化失败" : front.status(); }
        String shotMapStatus() { return shotMap == null ? "射界数据读取失败" : shotMap.description(); }
        void lowerFront(boolean lowered) { if (front != null) front.setLowered(lowered); }
        void forceLowerFront(boolean lowered) { if (front != null) front.forceLowered(lowered); }

        void collect(boolean enable) {
            if (collecting == enable) return;
            setFeeding(false);
            collecting = enable;
            if (intake != null) intake.set(collecting, false, false);
        }

        boolean update(double now, boolean braking, double turn, boolean triggerPressed) {
            if (!armed) return false;
            double dt = Maths.clip(now - lastUpdate, 0.001, 0.25);
            lastUpdate = now;
            if (turret != null) turret.sample(dt);
            boolean positionForAim = CornerAim.positionForAim(pose);
            aim = CornerAim.fromPose(shotMap, pose, braking);
            fallbackAim = aim == null && pose.headingUsable() && positionForAim
                    ? FieldTargets.fromPose(pose.forward, pose.left, pose.heading) : null;
            if (turret != null) {
                if (aim != null) {
                    CornerAim.track(turret, pose, aim, 0, turn, dt);
                } else if (fallbackAim != null && fallbackAim.distanceMm >= 0.1) {
                    double rate = new ShotMotion(pose).lineRate(fallbackAim.rightTiles, fallbackAim.forwardTiles);
                    turret.aim(pose, fallbackAim.goal, fallbackAim.bearing, rate, 0, turn, dt);
                } else {
                    turret.aim(pose, 0, 0, 0, 0, turn, dt);
                }
            }
            double targetRpm = Maths.clip(CornerAim.targetRpm(aim), 0, maxRpm);
            boolean noFireZone = CornerAim.noFireZone(pose);
            boolean trackingTurret = pose.headingUsable() && positionForAim;
            boolean startupReady = Double.isFinite(startedAt)
                    && now - startedAt >= Math.max(0.0, RobotConfig.AUTO_STARTUP_SPINUP_SECONDS);
            setFeeding(shot.update(triggerPressed, false, noFireZone, trackingTurret, false,
                    targetRpm, true, false, startupReady, Math.max(0.0, RobotConfig.AUTO_RPM_SETTLE_SECONDS)));
            if (triggerPressed) {
                shotStatus = noFireZone ? "禁射区，继续瞄准"
                        : feeding ? "正在供弹" : shot.waitingStatus();
            }
            return feeding;
        }

        void showStandby() {
            telemetry.addData("Pinpoint 右 / 前 / 车头", "%.2f / %.2f 格 / %+.1f°，%s",
                    -pose.left / RobotConfig.TILE_MM, pose.forward / RobotConfig.TILE_MM,
                    Math.toDegrees(pose.heading), pose.status);
            if (turret != null && turret.encoderReady()) {
                telemetry.addData("云台实际 / 目标", "%.1f° / %.1f°%s", Math.toDegrees(turret.angle),
                        Math.toDegrees(turret.target), turret.rewinding() ? "，回转中" : "");
            } else {
                telemetry.addData("云台", "编码器未就绪");
            }
            if (flywheel != null && flywheel.rpmCalibrated()) {
                telemetry.addData("飞轮实际 / 目标", "%.0f / %.0f RPM", flywheel.rpm(),
                        60 * flywheel.target / flywheel.ticksPerRev);
            }
        }

        void showVolley(String label, AutoVolley volley) {
            telemetry.addData("自动射击", label);
            telemetry.addData("供弹", "%.2f / %.2f 秒，%s", volley.feedSeconds(), volley.targetFeedSeconds(),
                    volley.finished() ? "本轮完成" : shotStatus);
            telemetry.addData("Pinpoint 右 / 前 / 车头", "%.2f / %.2f 格 / %+.1f°，%s",
                    -pose.left / RobotConfig.TILE_MM, pose.forward / RobotConfig.TILE_MM,
                    Math.toDegrees(pose.heading), pose.status);
            telemetry.addData("射界目标场地方向", aim != null ? String.format("%+.1f°", Math.toDegrees(aim.bearing))
                    : fallbackAim != null ? String.format("几何保底 %+.1f°", Math.toDegrees(fallbackAim.bearing))
                    : "保朝向 / 默认转速");
            if (turret != null && turret.encoderReady()) {
                telemetry.addData("云台实际 / 目标", "%.1f° / %.1f°%s", Math.toDegrees(turret.angle),
                        Math.toDegrees(turret.target), turret.rewinding() ? "，回转中" : "");
                if (pose.headingUsable()) telemetry.addData("实际发射场地方向", "%+.1f°",
                        Math.toDegrees(Maths.wrap(pose.heading + turret.boreRelative())));
            } else {
                telemetry.addData("云台", "编码器未就绪");
            }
            if (flywheel != null && flywheel.rpmCalibrated()) {
                telemetry.addData("飞轮实际 / 目标", "%.0f / %.0f RPM", flywheel.rpm(),
                        60 * flywheel.target / flywheel.ticksPerRev);
            }
        }

        void stopFeeding() {
            shot.releaseTrigger();
            setFeeding(false);
        }

        void stopShooting() {
            armed = false;
            shot.releaseTrigger();
            feeding = false;
            if (flywheel != null) flywheel.stop();
            if (turret != null) ControlLoop.safeStop("云台", () -> turret.teachingMode(false));
            if (intake != null) ControlLoop.safeStop("供弹", () -> intake.set(collecting, false, false));
        }

        void stop() {
            collecting = false;
            stopShooting();
            if (intake != null) intake.stop();
            if (front != null) front.stop();
        }

        private void setFeeding(boolean release) {
            if (feeding == release) return;
            feeding = flywheel != null && flywheel.feed(release);
            if (intake != null) intake.set(collecting, feeding, false);
        }

        private void initializeServo(HardwareMap map, String name, double position) {
            Servo servo = map.tryGet(Servo.class, name);
            if (servo != null) servo.setPosition(position);
            else telemetry.log().add("未配置舵机 %s", name);
        }

        private <T> T optional(String name, Supplier<T> factory) {
            try {
                return factory.get();
            } catch (RuntimeException unavailable) {
                telemetry.log().add("%s 不可用：%s", name, unavailable.getMessage());
                return null;
            }
        }
    }
}
