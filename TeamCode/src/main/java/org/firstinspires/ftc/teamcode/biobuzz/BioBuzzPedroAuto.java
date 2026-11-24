package org.firstinspires.ftc.teamcode.biobuzz;

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
import org.json.JSONException;

import java.io.IOException;

@Autonomous(name = "BIOBUZZ Pedro 吸两组设三组2T", group = "BIOBUZZ")
public final class BioBuzzPedroAuto extends LinearOpMode {
    private static final double POSITION_TOLERANCE_INCHES = 1.5;
    private static final double HEADING_TOLERANCE = Math.toRadians(7);
    private static final double STALL_SECONDS = 4.5;
    private static final double STALL_SPEED_INCHES_PER_SECOND = 0.5;
    private static final double STALL_YAW_RATE = Math.toRadians(4);
    private static final double PINPOINT_PAUSE_SECONDS = 0.25;
    private static final long TIMEOUT_BRAKE_MS = 100;
    private static final long FRONT_LOWER_SETTLE_MS = 180;

    private enum LegResult { REACHED, TIMED_OUT, STOPPED }

    @Override public void runOpMode() throws InterruptedException {
        BatteryVoltage battery = new BatteryVoltage(hardwareMap);
        PinpointPose pinpoint = new PinpointPose(hardwareMap, battery);
        Drive drive = null;
        PedroDrive mecanum = null;
        PedroAutoMechanisms mechanisms = null;
        String status;
        try (ControlLoop loop = new ControlLoop(hardwareMap)) {
            mechanisms = new PedroAutoMechanisms(hardwareMap, telemetry, battery, pinpoint);
            PedroRoute route = PedroRoute.load(hardwareMap.appContext.getAssets());
            if (route.legs.size() != 8) throw new JSONException("机构流程需要 8 段 PP 路径");
            drive = new Drive(hardwareMap);
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
                if (!initAnchored) initAnchored = PedroPinpointLocalizer.anchorRouteOrigin(pinpoint, route.start);
                telemetry.addData("Pinpoint", pinpoint.status);
                telemetry.addData("FR / FL", mechanisms.frontStatus());
                telemetry.addData("射击数据", mechanisms.shotMapStatus());
                telemetry.addData("起点 X / Y / 车头", "%.2f / %.2f in / %.0f°",
                        route.start.x(), route.start.y(), Math.toDegrees(route.start.heading()));
                telemetry.addData("第 1 段末端车头", "%.0f°", Math.toDegrees(firstEnd.heading()));
                telemetry.addData("终点 X / Y / 车头", "%.2f / %.2f in / %.0f°",
                        end.x(), end.y(), Math.toDegrees(end.heading()));
                telemetry.addLine("PP 场地图角度 | 0° 朝右 · 90° 朝上 · 180° 朝左 · 270° 朝下");
                telemetry.addLine("保持底盘静止，把云台轻摆到最右端后 START");
                telemetry.update();
                sleep(25);
            }
            if (isStopRequested()) return;
            loop.beginFrame();
            localizer.sample();
            if (!initAnchored) PedroPinpointLocalizer.anchorRouteOrigin(pinpoint, route.start);
            if (!pinpoint.positionValid) throw new PedroPinpointLocalizer.PositionUnavailable(pinpoint.status);
            mechanisms.start();
            int timedOut = 0;
            boolean rejoin = false;
            for (int index = 0; index < route.legs.size(); index++) {
                PedroRoute.Leg leg = route.legs.get(index);
                int number = index + 1;
                if (!waitAtStop(leg.waitBefore, localizer, loop, mechanisms)) return;
                if (number == 2 || number == 5) mechanisms.collect(true);
                String name = String.format("%d/%d %s", number, route.legs.size(), leg.name);
                LegResult result = followLeg(follower, mecanum, leg, name, rejoin, number == 8, loop, mechanisms);
                if (result == LegResult.STOPPED) return;
                if (number == 7) mechanisms.collect(false);
                if (number == 8) mechanisms.lowerFront(false);
                rejoin = result == LegResult.TIMED_OUT;
                if (rejoin) {
                    timedOut++;
                    Pose target = leg.path.endPose();
                    Pose measured = follower.pose();
                    telemetry.log().add("%s 未到位，持续低速，超时跳过底盘。偏差 %.2f in / %.1f°", name,
                            measured.distance(target), Math.toDegrees(Maths.wrap(target.heading() - measured.heading())));
                    telemetry.update();
                    if (!waitAtStop(TIMEOUT_BRAKE_MS, localizer, loop, mechanisms)) return;
                }
                if (number == 1 || number == 3 || number == 7) {
                    if (!shootAtStop(leg.waitAfter, name, localizer, loop, mechanisms)) return;
                } else if (number == 2 || result == LegResult.REACHED) {
                    long waitMs = leg.waitAfter;
                    if (number == 4) {
                        mechanisms.lowerFront(true);
                        waitMs = Math.max(waitMs, FRONT_LOWER_SETTLE_MS);
                    }
                    if (!waitAtStop(waitMs, localizer, loop, mechanisms)) return;
                }
                if (number == 2) mechanisms.collect(false);
                if (number == 7) mechanisms.stopShooting();
            }
            status = timedOut == 0 ? "路径完成" : String.format("路线执行结束，超时跳过 %d 段", timedOut);
        } catch (IOException | JSONException | RuntimeException failure) {
            status = "自动停止：" + failure.getMessage();
        } finally {
            if (mecanum != null) HardwareStop.attempt("Pedro 底盘", mecanum::stop);
            if (drive != null) drive.stop();
            if (mechanisms != null) mechanisms.stop();
        }

        while (!isStopRequested()) {
            pinpoint.update();
            if (pinpoint.hasPositionAnchor()) {
                telemetry.addData("Pinpoint 右 / 前 / 车头", "%.2f / %.2f 格 / %+.1f°",
                        -pinpoint.left / RobotConfig.TILE_MM, pinpoint.forward / RobotConfig.TILE_MM,
                        Math.toDegrees(pinpoint.heading));
            }
            telemetry.addData("Pinpoint", pinpoint.status);
            telemetry.addLine(status);
            telemetry.update();
            sleep(50);
        }
    }

    private boolean waitAtStop(long millis, PedroPinpointLocalizer localizer, ControlLoop loop,
                               PedroAutoMechanisms mechanisms) throws InterruptedException {
        return holdAtStop(millis, null, null, localizer, loop, mechanisms);
    }

    private boolean shootAtStop(long millis, String name, PedroPinpointLocalizer localizer,
                                ControlLoop loop, PedroAutoMechanisms mechanisms) throws InterruptedException {
        AutoVolley volley = new AutoVolley(Maths.time(), millis / 1000.0);
        boolean active;
        try {
            active = holdAtStop(millis, name, volley, localizer, loop, mechanisms);
        } finally {
            mechanisms.stopFeeding();
        }
        if (active) telemetry.log().add("%s 供弹 %.2f 秒%s", name, volley.feedSeconds(),
                volley.finished() ? "" : "，射击等待超时");
        return active;
    }

    private boolean holdAtStop(long millis, String name, AutoVolley volley,
                               PedroPinpointLocalizer localizer, ControlLoop loop, PedroAutoMechanisms mechanisms)
            throws InterruptedException {
        double started = Maths.time();
        double until = started + millis / 1000.0;
        while (opModeIsActive()) {
            double frame = loop.beginFrame();
            boolean triggerPressed = volley != null && volley.requesting(frame);
            if (frame >= until && !triggerPressed) break;
            localizer.sample();
            if (isStopRequested()) return false;
            boolean feeding = mechanisms.update(frame, true, 0, triggerPressed);
            if (volley != null) {
                volley.recordOutput(feeding);
                mechanisms.showVolley(name, volley);
                telemetry.update();
            }
            waitForNextFrame(loop, frame);
        }
        return opModeIsActive();
    }

    private LegResult followLeg(Follower follower, PedroDrive drive, PedroRoute.Leg leg, String name, boolean rejoin,
                                boolean park, ControlLoop loop, PedroAutoMechanisms mechanisms) throws InterruptedException {
        Pose target = leg.path.endPose();
        try {
            double stalledSince = Double.NaN;
            double invalidSince = Double.NaN;
            boolean following = false;
            while (opModeIsActive()) {
                double frame = loop.beginFrame();
                telemetry.addData("路径", name);
                try {
                    if (!following) {
                        follower.localizer.update();
                        Pose measured = follower.pose();
                        if (rejoin && measured.distance(target) <= POSITION_TOLERANCE_INCHES) {
                            follower.hold(target);
                        } else {
                            Path path = rejoin ? leg.fromMeasuredPose(measured) : leg.path;
                            follower.follow(path);
                        }
                        following = true;
                    } else {
                        follower.update();
                    }
                    if (isStopRequested()) return LegResult.STOPPED;
                    invalidSince = Double.NaN;
                } catch (PedroPinpointLocalizer.PositionUnavailable failure) {
                    follower.drivetrain.stop();
                    stalledSince = Double.NaN;
                    if (!Double.isFinite(invalidSince)) invalidSince = frame;
                    if (frame - invalidSince >= PINPOINT_PAUSE_SECONDS) throw failure;
                    if (isStopRequested()) return LegResult.STOPPED;
                    mechanisms.update(frame, true, 0, false);
                    telemetry.addData("Pinpoint", "暂停，等待有效定位");
                    telemetry.update();
                    waitForNextFrame(loop, frame);
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
                        && activePath.curve.remainingDistance(follower.parametricCompletion()) <= POSITION_TOLERANCE_INCHES);
                if (nearPathEnd && distance <= POSITION_TOLERANCE_INCHES && turn <= HEADING_TOLERANCE
                        && speed <= 4 && yawRate <= Math.toRadians(20)) return LegResult.REACHED;
                if (speed < STALL_SPEED_INCHES_PER_SECOND && yawRate < STALL_YAW_RATE) {
                    if (!Double.isFinite(stalledSince)) stalledSince = frame;
                    if (frame - stalledSince >= STALL_SECONDS) return LegResult.TIMED_OUT;
                    telemetry.addData("未到位，低速计时", "%.1f / %.1f 秒", frame - stalledSince, STALL_SECONDS);
                } else {
                    stalledSince = Double.NaN;
                }
                telemetry.addData("终点误差", "%.1f in / %.0f°", distance, Math.toDegrees(turn));
                telemetry.addData("底盘实测速度", "%.2f in/s / %.1f°/s", speed, Math.toDegrees(yawRate));
                telemetry.update();
                waitForNextFrame(loop, frame);
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

    private void waitForNextFrame(ControlLoop loop, double frame) throws InterruptedException {
        long remaining = loop.remainingMillis(frame);
        if (remaining > 0) sleep(remaining);
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
            // Pedro stores the normalized powers in FL, FR, BL, BR order.
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
        // Initial gains. Drive speeds use the planner estimates; deceleration uses the robot's existing values.
        double forwardDecel = RobotConfig.BRAKE_FORWARD_MM_S2 / PedroPinpointLocalizer.MM_PER_INCH;
        double strafeDecel = RobotConfig.BRAKE_SIDE_MM_S2 / PedroPinpointLocalizer.MM_PER_INCH;
        return new ForesightConfig(c -> {
            c.forwardTranslational.set(Controller.proportional(0.15));
            c.strafeTranslational.set(Controller.proportional(0.18));
            c.headingFeedback.set(Controller.proportional(1.6));
            c.coast.set(Controller.sum(Controller.staticFeedforward(0.08), Controller.proportional(0.013)));
            c.brake.set(Controller.proportionalFeedforward(0.013));
            c.linearBrakeCoefficients.set(Matrix.diag(0.04, 0.04));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.5 / forwardDecel, 0.5 / strafeDecel));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.045, 0.005));
            c.maxAchievableForwardVelocity.set(75.0);
            c.maxAchievableStrafeVelocity.set(65.0);
            c.naturalForwardDeceleration.set(forwardDecel);
            c.naturalStrafeDeceleration.set(strafeDecel);
            c.maxPathSpeed.set(1.0);
            c.maxDecelerationConstraint.set(50.0);
            c.pathSkip.set(false);
        });
    }
}
