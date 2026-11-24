package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotStateStorage;
import org.firstinspires.ftc.teamcode.biobuzz.core.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.core.ControlLoop;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.localization.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming.CornerAim;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming.FieldTargets;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming.ShotMotion;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.Aiming.ShotSequence;
import org.firstinspires.ftc.teamcode.biobuzz.shooting.ShotMap;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Drive.TurnLinearity;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake.TaskServos;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.RearStatusLed;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Turret;

import java.util.function.Supplier;

/**
 * Unified corner-mode TeleOp for symmetrical field play.
 * Inherits Pinpoint pose & turret home tick automatically after Autonomous without touching the robot.
 */
@TeleOp(name = "Biobuzz角落出发", group = "BIOBUZZ")
public final class BioBuzzTeleOp extends LinearOpMode {
    private static final double DISPLAY_PERIOD = 0.25;
    private double driverYawZero;
    private boolean useGoals = true;
    private boolean lastG1A, lastG1X, lastG1Y, lastG1DpadUp, lastG1DpadLeft, lastG1DpadRight;
    private boolean lastG2A, lastG2X, lastG2Y, lastG2LB, lastG2RB;
    private boolean lastG2DpadUp, lastG2DpadDown, lastG2DpadLeft, lastG2DpadRight;

    @Override public void runOpMode() throws InterruptedException {
        CalibrationStore store = new CalibrationStore(hardwareMap.appContext);
        TurnLinearity turnLinearity = TurnLinearity.load(store);
        ShotMap shotMap = new ShotMap(store);
        BatteryVoltage battery = new BatteryVoltage(hardwareMap);
        Drive drive = optional("底盘", () -> new Drive(hardwareMap));
        Turret turret = optional("云台", () -> new Turret(hardwareMap, store, battery));
        Flywheel shooter = optional("飞轮", () -> new Flywheel(hardwareMap, store, battery));
        Intake intake = optional("吸取", () -> new Intake(hardwareMap));
        TaskServos taskServos = optional("FR/FL", () -> new TaskServos(hardwareMap));
        PinpointPose pose = new PinpointPose(hardwareMap, battery);
        pose.attachExisting();
        ShotSequence shot = new ShotSequence(turret, shooter, intake, true);

        Servo pitch = hardwareMap.tryGet(Servo.class, RobotConfig.PITCH_SERVO);
        RearStatusLed lights = RearStatusLed.open(hardwareMap);
        boolean startedTeleOp = false;
        try (ControlLoop loop = new ControlLoop(hardwareMap)) {
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
            if (pitch != null) pitch.setPosition(RobotConfig.ST_HOME);
            boolean inheritedTurret = RobotStateStorage.hasTurretHome();
            if (turret != null) {
                turret.teachingMode(!inheritedTurret);
                if (inheritedTurret && turret.calibrated()) {
                    turret.homeAtRight(RobotStateStorage.turretHomeTick);
                }
            }
            lights.show(RearStatusLed.State.INIT);

            boolean poseInitialized = false;
            double lastInitTime = Maths.time();
            while (opModeInInit()) {
                double now = loop.beginFrame();
                double initDt = Maths.clip(now - lastInitTime, 0.005, 0.25);
                lastInitTime = now;
                battery.update(now);
                poseInitialized = syncTeleOpPose(pose, poseInitialized);
                if (inheritedTurret && turret != null && turret.isHomed()) turret.sample(initDt);
                showBattery(battery);
                if (store.turretStale()) telemetry.addLine("<font color='#ffcc33'>云台参数已过期，请重跑对应校准</font>");
                if (RobotStateStorage.hasAutoPose() || inheritedTurret) {
                    telemetry.addLine("<font color='#55ff99'>已继承自动位姿与云台零位，直接 START（无需碰车）</font>");
                } else {
                    telemetry.addLine("未检测到自动衔接：把云台轻摆到最右、车放角落后 START");
                }
                telemetry.addLine("主/副手 Y：回角落重置坐标 | 副手 A：云台锁死并回正前方");
                if (pose.positionValid) {
                    telemetry.addData("当前位姿 右 / 前 / 车头", "%.2f / %.2f 格 / %+.1f°",
                            -pose.left / RobotConfig.TILE_MM, pose.forward / RobotConfig.TILE_MM, Math.toDegrees(pose.heading));
                }
                telemetry.addData("Pinpoint", pose.status);
                if (taskServos != null) telemetry.addData("FR / FL", taskServos.status());
                if (!lights.configured()) telemetry.addData("尾灯", "未配置 LED");
                if (turret == null || !turret.calibrated()) telemetry.addData("云台", "先运行 02 自测");
                else {
                    telemetry.addData("云台接续", inheritedTurret && turret.isHomed()
                            ? String.format("已继承自动零位（当前 %+.1f°）", Math.toDegrees(turret.boreRelative()))
                            : String.format("待采右端（INIT 相对 %+.1f° / 跨度 %.1f°）",
                            Math.toDegrees(turret.initRelativeAngle()), Math.toDegrees(turret.span())));
                    showTurretCalibration(turret);
                }
                telemetry.addData("飞轮", shooter != null && shooter.rpmCalibrated()
                        ? "START 后跟随 06/模型转速，缺数据时使用默认转速"
                        : "先运行 04，自测 RPM 比例和转速曲线");
                telemetry.addData("射击数据", shotMap.description());
                if (shooter != null) telemetry.addData("飞轮转向 m-7 / m-8", "%s / %s",
                        shooter.leftDirection(), shooter.encoderDirection());
                if (shooter != null && shooter.rpmCalibrated()) {
                    showFlywheelPid(shooter);
                    if (!shooter.voltageModelCalibrated()) {
                        telemetry.addLine("<font color='#ffcc33'>飞轮仍在使用旧占空比曲线，建议重跑 04 校准 12V 前馈</font>");
                    }
                }
                telemetry.addData("转向摇杆线性映射", turnLinearity.summary());
                telemetry.update();
                sleep(25);
            }
            if (isStopRequested()) return;

            startedTeleOp = true;
            loop.beginFrame();
            if (turret != null) {
                turret.teachingMode(false);
                if (turret.calibrated()) {
                    if (RobotStateStorage.hasTurretHome()) {
                        turret.homeAtRight(RobotStateStorage.turretHomeTick);
                    } else {
                        turret.homeAtRight();
                    }
                }
            }
            if (!RobotStateStorage.hasAutoPose() && pose.canAnchorPosition()) {
                pose.anchorManualCorner();
                poseInitialized = true;
            } else {
                poseInitialized = syncTeleOpPose(pose, poseInitialized);
            }
            if (isStopRequested()) return;

            lastG1A = gamepad1.a;
            lastG1X = gamepad1.x;
            lastG1Y = gamepad1.y;
            lastG1DpadUp = gamepad1.dpad_up;
            lastG1DpadLeft = gamepad1.dpad_left;
            lastG1DpadRight = gamepad1.dpad_right;

            lastG2A = gamepad2.a;
            lastG2X = gamepad2.x;
            lastG2Y = gamepad2.y;
            lastG2LB = gamepad2.left_bumper;
            lastG2RB = gamepad2.right_bumper;
            lastG2DpadUp = gamepad2.dpad_up;
            lastG2DpadDown = gamepad2.dpad_down;
            lastG2DpadLeft = gamepad2.dpad_left;
            lastG2DpadRight = gamepad2.dpad_right;

            boolean lastRpmLock = gamepad1.right_stick_button && !gamepad1.back;
            boolean lastG1TrimReset = gamepad1.left_stick_button;
            boolean lastTurretUnlock = (gamepad1.back && gamepad1.right_stick_button) || gamepad2.right_stick_button;
            boolean rpmLockEnabled = false, turretUnlocked = false, turretLockedZero = false;
            double yawTrim = 0;
            double maxRpm = shooter != null ? shooter.maxTargetRpm() : 0;
            double targetRpm = Math.min(RobotConfig.FLYWHEEL_TARGET_RPM, maxRpm);
            double rpmTrim = 0;

            lights.show(RearStatusLed.State.DRIVE);
            boolean lastTriggerPressed = false;
            boolean blockerOpen = false;
            boolean wasAimedReady = false;
            double previous = Maths.time();
            double nextPoseLog = previous, nextDisplay = previous, nextDashboard = previous;
            double lastFireRumble = previous;
            double loopHz = 0;
            FtcDashboard dashboard = FtcDashboard.getInstance();
            double rightLimitSince = Double.NaN;
            boolean rightLimitWarning = false;

            while (opModeIsActive()) {
                double loopStart = loop.beginFrame();
                battery.update(loopStart);
                boolean triggerPressed = gamepad1.right_trigger > 0.10f;
                boolean triggerChanged = triggerPressed != lastTriggerPressed;
                if (triggerChanged) {
                    RobotLog.ii("BioBuzzShot", "RT %s", triggerPressed ? "pressed" : "released");
                    lastTriggerPressed = triggerPressed;
                }
                double f = Maths.deadband(-gamepad1.left_stick_y);
                double l = Maths.deadband(-gamepad1.left_stick_x);
                double rawTurn = Maths.deadband(-gamepad1.right_stick_x);
                boolean driveIdle = f == 0 && l == 0 && rawTurn == 0;

                poseInitialized = syncTeleOpPose(pose, poseInitialized);
                if (isStopRequested()) break;

                // Driver & Deputy Y: reset coordinates to manual corner
                boolean g1Y = gamepad1.y, g2Y = gamepad2.y;
                if ((g1Y && !lastG1Y) || (g2Y && !lastG2Y)) {
                    boolean reanchored = pose.anchorManualCorner();
                    if (isStopRequested()) break;
                    if (reanchored) {
                        driverYawZero = 0;
                        if (turret != null && !turretLockedZero && !turretUnlocked) turret.stop();
                        gamepad1.rumble(150);
                        gamepad2.rumble(150);
                    }
                }
                lastG1Y = g1Y;
                lastG2Y = g2Y;

                // Driver X: toggle auto-aim vs ground-hold
                if (gamepad1.x && !lastG1X) {
                    useGoals = !useGoals;
                    if (turret != null && !turretLockedZero) turret.stop();
                }
                lastG1X = gamepad1.x;

                boolean field = pose.headingUsable();
                // Driver A: headless forward reset
                if (gamepad1.a && !lastG1A && field && pose.headingValid) {
                    driverYawZero = pose.heading;
                    gamepad1.rumble(100);
                }
                lastG1A = gamepad1.a;

                double timestamp = Maths.time();
                double rawDt = timestamp - previous;
                if (rawDt > 0) loopHz += 0.1 * (1.0 / rawDt - loopHz);
                double dt = Maths.clip(rawDt, 0.001, 0.25);
                previous = timestamp;
                if (turret != null) turret.sample(dt);

                boolean reverseRequested = gamepad1.right_bumper;
                boolean triggerHeld = triggerPressed && !reverseRequested;

                boolean rpmLockPressed = gamepad1.right_stick_button && !gamepad1.back;
                if (rpmLockPressed && !lastRpmLock) rpmLockEnabled = !rpmLockEnabled;
                lastRpmLock = rpmLockPressed;

                // Deputy A: lock turret at forward zero (boreRelative = 0°)
                boolean g2A = gamepad2.a;
                if (g2A && !lastG2A) {
                    turretLockedZero = !turretLockedZero;
                    if (turretLockedZero) {
                        turretUnlocked = false;
                        gamepad2.rumble(150);
                    } else {
                        if (turret != null) turret.stop();
                        gamepad2.rumble(80);
                    }
                }
                lastG2A = g2A;

                // Emergency turret decouple: G1 Back+R3 or G2 R3
                boolean turretUnlockPressed = (gamepad1.back && gamepad1.right_stick_button) || gamepad2.right_stick_button;
                if (turretUnlockPressed && !lastTurretUnlock) {
                    turretUnlocked = !turretUnlocked;
                    if (turretUnlocked) turretLockedZero = false;
                    if (turret != null) turret.stop();
                }
                lastTurretUnlock = turretUnlockPressed;

                // Driver dpad_up: FR/FL servo toggle
                boolean g1Up = gamepad1.dpad_up;
                if (g1Up && !lastG1DpadUp && !gamepad1.back && taskServos != null) {
                    taskServos.toggle();
                }
                lastG1DpadUp = g1Up;

                // Deputy yaw trim: dpad left/right (±G2_YAW_FINE_DEG), LB/RB (±G2_YAW_LARGE_DEG), left positive
                boolean g2Left = gamepad2.dpad_left, g2Right = gamepad2.dpad_right;
                boolean g2LB = gamepad2.left_bumper, g2RB = gamepad2.right_bumper;
                if (g2Left && !lastG2DpadLeft) yawTrim += Math.toRadians(RobotConfig.G2_YAW_FINE_DEG);
                if (g2Right && !lastG2DpadRight) yawTrim -= Math.toRadians(RobotConfig.G2_YAW_FINE_DEG);
                if (g2LB && !lastG2LB) yawTrim += Math.toRadians(RobotConfig.G2_YAW_LARGE_DEG);
                if (g2RB && !lastG2RB) yawTrim -= Math.toRadians(RobotConfig.G2_YAW_LARGE_DEG);
                lastG2DpadLeft = g2Left;
                lastG2DpadRight = g2Right;
                lastG2LB = g2LB;
                lastG2RB = g2RB;

                // Deputy RPM fine trim: dpad up/down (±G2_RPM_FINE_STEP)
                boolean g2Up = gamepad2.dpad_up, g2Down = gamepad2.dpad_down;
                double rpmStep = 0;
                if (g2Up && !lastG2DpadUp) rpmStep += RobotConfig.G2_RPM_FINE_STEP;
                if (g2Down && !lastG2DpadDown) rpmStep -= RobotConfig.G2_RPM_FINE_STEP;
                lastG2DpadUp = g2Up;
                lastG2DpadDown = g2Down;

                // Driver RPM coarse trim: dpad left/right (±FLYWHEEL_RPM_STEP)
                boolean g1Slower = gamepad1.dpad_left && !lastG1DpadLeft;
                boolean g1Faster = gamepad1.dpad_right && !lastG1DpadRight;
                if (g1Faster) rpmStep += RobotConfig.FLYWHEEL_RPM_STEP;
                if (g1Slower) rpmStep -= RobotConfig.FLYWHEEL_RPM_STEP;
                lastG1DpadLeft = gamepad1.dpad_left;
                lastG1DpadRight = gamepad1.dpad_right;

                // Reset trims: G1 L3 or G2 X
                boolean g1TrimReset = gamepad1.left_stick_button;
                boolean g2X = gamepad2.x;
                if ((g1TrimReset && !lastG1TrimReset) || (g2X && !lastG2X)) {
                    yawTrim = 0;
                    rpmTrim = 0;
                    if (g2X && !lastG2X) gamepad2.rumble(100);
                }
                lastG1TrimReset = g1TrimReset;
                lastG2X = g2X;

                double driveLimit = battery.low() ? BatteryVoltage.LOW_DRIVE_LIMIT : 1;
                double turn = turnLinearity.mapDuty(rawTurn, f, l, battery, driveLimit);
                double effective = 0;
                if (drive != null) {
                    effective = field ? drive.field(f, l, turn, pose.heading - driverYawZero, driveLimit)
                            : drive.robot(f, l, turn, driveLimit);
                }

                boolean noFireZone = CornerAim.noFireZone(pose);
                boolean automatic = useGoals || noFireZone;
                boolean positionForAim = CornerAim.positionForAim(pose);
                CornerAim sample = CornerAim.fromPose(shotMap, pose, f == 0 && l == 0);
                FieldTargets fallbackAim = sample == null && automatic && field && positionForAim
                        ? FieldTargets.fromPose(pose.forward, pose.left, pose.heading) : null;

                double baseRpm = CornerAim.targetRpm(sample);
                double firingRpm = Maths.clip(baseRpm + rpmTrim, 0, maxRpm);
                if (rpmStep != 0) {
                    firingRpm = Maths.clip(firingRpm + rpmStep, 0, maxRpm);
                    rpmTrim = firingRpm - baseRpm;
                }
                targetRpm = firingRpm;

                if (turret != null && !turretUnlocked) {
                    if (turretLockedZero) {
                        turret.lockAtForward(dt);
                    } else if (automatic && sample != null) {
                        CornerAim.track(turret, pose, sample, yawTrim, effective, dt);
                    } else if (fallbackAim != null && fallbackAim.distanceMm >= 0.1) {
                        double rate = new ShotMotion(pose).lineRate(fallbackAim.rightTiles, fallbackAim.forwardTiles);
                        turret.aim(pose, fallbackAim.goal, fallbackAim.bearing, rate, yawTrim, effective, dt);
                    } else {
                        turret.aim(pose, 0, 0, 0, yawTrim, effective, dt);
                    }
                } else if (turret != null && !turret.motorResponding()) {
                    turret.stop();
                }

                boolean nearStalledRight = turret != null && !turretUnlocked && turret.encoderReady()
                        && turret.angle > RobotConfig.TURRET_MARGIN && turret.angle < Math.toRadians(9)
                        && turret.target <= RobotConfig.TURRET_MARGIN + Math.toRadians(0.5)
                        && Math.abs(turret.speed) < 0.10 && turret.output() < -0.04;
                if (nearStalledRight) {
                    if (!Double.isFinite(rightLimitSince)) rightLimitSince = timestamp;
                    if (!rightLimitWarning && timestamp - rightLimitSince >= 0.40) {
                        rightLimitWarning = true;
                        RobotLog.ww("BioBuzzTurret", "Right limit stall: measured angle %.2f degrees",
                                Math.toDegrees(turret.angle));
                    }
                } else {
                    rightLimitSince = Double.NaN;
                    rightLimitWarning = false;
                }

                boolean releaseShot = shot.update(triggerPressed, reverseRequested, noFireZone,
                        !turretUnlocked && field && positionForAim, turretLockedZero, targetRpm, rpmLockEnabled);

                boolean firing = shooter != null && shooter.feed(releaseShot);
                if (intake != null) intake.set(gamepad1.left_bumper && !triggerPressed, firing, reverseRequested);
                if (blockerOpen != firing) {
                    RobotLog.ii("BioBuzzShot", "SB %s", firing ? "opened" : "closed");
                    blockerOpen = firing;
                }

                boolean aimedReady = turret != null && !turretUnlocked && !turretLockedZero
                        && turret.aimReady() && automatic && (sample != null || fallbackAim != null);
                if (aimedReady && !wasAimedReady && !triggerHeld) {
                    gamepad1.rumble(0.8, 0.8, 120);
                }
                wasAimedReady = aimedReady;
                if (firing && timestamp - lastFireRumble >= 0.25) {
                    gamepad1.rumble(0.4, 0.4, 80);
                    lastFireRumble = timestamp;
                }

                if (dashboard != null && timestamp >= nextDashboard) {
                    nextDashboard = timestamp + 0.067;
                    try {
                        publishDashboard(dashboard, battery, turret, shooter, firing, rpmLockEnabled, loopHz);
                    } catch (RuntimeException ignored) {
                        // Dashboard failures must not disturb the robot control loop.
                    }
                }
                if (triggerChanged || ((triggerPressed || gamepad1.back || driveIdle) && timestamp >= nextPoseLog)) {
                    logPose(pose, turret, driveIdle, triggerPressed, firing);
                    nextPoseLog = timestamp + (triggerPressed || gamepad1.back ? 0.25 : 2);
                }
                if (timestamp >= nextDisplay) {
                    nextDisplay = timestamp + DISPLAY_PERIOD;
                    showBattery(battery);
                    if (rightLimitWarning) telemetry.addLine("云台右端停滞，请核对零位");
                    telemetry.addData("发射模式", rpmLockEnabled ? "锁飞轮转速" : "高速直射");
                    telemetry.addData("微调", "云台 %+.1f° | 转速 %+.0f RPM", Math.toDegrees(yawTrim), rpmTrim);
                    if (taskServos != null) telemetry.addData("FR / FL", taskServos.status());
                    if (reverseRequested && intake != null) telemetry.addData("供弹通道", "反转");
                    if (noFireZone) telemetry.addData("射击", "禁射区，继续瞄准");
                    else if (triggerHeld) {
                        if (firing) telemetry.addData("射击", "已放行，松开 RT 结束");
                        else {
                            String waiting = shot.waitingStatus();
                            if (waiting != null) telemetry.addData("射击", waiting);
                        }
                    }
                    if (shooter != null) {
                        telemetry.addData("飞轮", shooter.faulted() ? "编码器故障，请停机检查"
                                : !shooter.rpmCalibrated() ? "先运行 04，自测 RPM 比例和转速曲线"
                                : String.format("%.0f / %.0f RPM", shooter.speed * 60 / shooter.ticksPerRev,
                                        shooter.target * 60 / shooter.ticksPerRev));
                        if (gamepad1.back && shooter.rpmCalibrated()) {
                            showFlywheelPid(shooter);
                            telemetry.addData("飞轮前馈 / 积分 / 实际输出", "%.2f V / %+.2f V / %.2f V",
                                    shooter.feedforwardVolts(), shooter.integralVolts(), shooter.deliveredVolts());
                            telemetry.addData("飞轮偏差 / 转速上限", "%+.0f / %.0f RPM",
                                    (shooter.speed - shooter.target) * 60 / shooter.ticksPerRev, maxRpm);
                        }
                    }
                    telemetry.addData("底盘", field ? "无头" : "车头模式");
                    if (driveLimit < 1) telemetry.addData("底盘输出上限", "%.0f%%", 100 * driveLimit);
                    telemetry.addData("右 / 前 / 车头", "%.2f / %.2f 格 / %+.1f°", -pose.left / RobotConfig.TILE_MM,
                            pose.forward / RobotConfig.TILE_MM, Math.toDegrees(pose.heading));
                    telemetry.addData("Pinpoint", pose.status);
                    if (turretLockedZero) {
                        telemetry.addData("目标", "副手 A 锁死正前零位（再按 A 恢复自瞄）");
                    } else if (!automatic) {
                        telemetry.addData("目标", "手动锁方向（X 恢复自瞄）");
                    } else if (sample != null) {
                        telemetry.addData("目标", "射界模型 / %+.1f° / %.0f RPM%s",
                                Math.toDegrees(sample.bearing), sample.rpm, RobotConfig.ENABLE_MOVING_LEAD ? " [跑打]" : "");
                    } else if (fallbackAim != null) {
                        telemetry.addData("目标", "几何保底 %d / %+.1f°", fallbackAim.goal, Math.toDegrees(fallbackAim.bearing));
                    } else {
                        telemetry.addData("目标", "等待定位");
                    }
                    if (gamepad1.back) {
                        telemetry.addData("控制频率", "%.0f Hz", loopHz);
                        if (rawTurn != 0) {
                            telemetry.addData("转向 摇杆 / 映射输出 / 角速度", "%+.2f → %+.2f / %+.0f°/s",
                                    rawTurn, effective, Math.toDegrees(pose.yawRate));
                        }
                        if (pose.faults != 0) telemetry.addData("最近定位异常", "%s，累计 %d", pose.lastFault, pose.faults);
                        if (pose.badReads != 0) {
                            telemetry.addData("Pinpoint 固件 / 坏读 / 恢复 / 连续坏读", "V%d / %d / %d / %d",
                                    pose.firmwareVersion, pose.badReads, pose.recoveredReads, pose.badReadStreak);
                        }
                    }
                    if (pose.requiresReanchor() || !pose.hasPositionAnchor()) {
                        telemetry.addLine("定位需要对齐，回角落按 Y");
                    }
                    if (turret != null) {
                        String tStatus = turretUnlocked ? "云台解耦"
                                : turretLockedZero ? (turret.atForwardZero() ? "已锁死正前零位" : "正在回正前零位")
                                : !turret.calibrated() ? "先运行 02 自测"
                                : !turret.isHomed() ? "零位读取失败，重 INIT 摆到最右"
                                : !turret.encoderReady() || !turret.motorResponding() ? "读数或电机恢复中"
                                : !field ? "等待 Pinpoint 航向"
                                : turret.rewindPredicted() ? "预测回转，等待跨接缝"
                                : turret.rewinding() ? "PID 回转"
                                : !turret.targetReachable() ? "目标超出行程"
                                : String.format("发射朝向 %+.1f°", Math.toDegrees(Maths.wrap(pose.heading + turret.boreRelative())));
                        telemetry.addData("云台", tStatus);
                        if (gamepad1.back) {
                            showTurretCalibration(turret);
                            telemetry.addData("云台目标 / 行程角 / 速度 / 功率", "%.1f° / %.1f° / %+.0f°/s / %+.2f",
                                    Math.toDegrees(turret.target), Math.toDegrees(turret.angle),
                                    Math.toDegrees(turret.speed), turret.output());
                        }
                        if (field && turret.calibrated() && turret.encoderReady()) {
                            telemetry.addData("云台误差 / 入闸阈值", "%+.1f° / %.1f°",
                                    Math.toDegrees(turret.worldError), Math.toDegrees(turret.shotAimTolerance()));
                        }
                    }
                    telemetry.addLine("主手：LB 吸 | RT 射 | RB 退 | A 无头归零 | X 锁向 | Y 重置 | 方向左右 ±100RPM | R3 锁速");
                    telemetry.addLine("副手：上下 ±25RPM | 左右 ±0.5° | LB/RB ±5° | Y 重置坐标 | A 锁死回正前 | X 清微调 | R3 解耦");
                    telemetry.update();
                }

                if (!triggerPressed && gamepad1.right_trigger <= 0.10f && !isStopRequested()) {
                    boolean turretFault = !turretUnlocked && (turret == null || !turret.calibrated()
                            || !turret.encoderReady() || !turret.motorResponding());
                    boolean mechanismFault = drive == null || intake == null || shooter == null || turretFault
                            || shooter.faulted() || !shooter.rpmCalibrated() || !shooter.hasBlocker();
                    lights.update(rearState(mechanismFault || !field || !positionForAim, reverseRequested,
                            noFireZone, turretUnlocked, turretLockedZero, gamepad1.left_bumper, aimedReady, rpmLockEnabled));
                }
            }
        } finally {
            if (startedTeleOp) RobotStateStorage.clear();
            if (shooter != null) shooter.stop();
            if (intake != null) intake.stop();
            if (drive != null) drive.stop();
            if (turret != null) ControlLoop.safeStop("云台", () -> turret.teachingMode(false));
            ControlLoop.safeStop("尾灯", lights::stop);
        }
    }

    private static boolean syncTeleOpPose(PinpointPose pose, boolean poseInitialized) {
        pose.update();
        if (!poseInitialized && !pose.hasPositionAnchor() && pose.canAnchorPosition()) {
            if (RobotStateStorage.hasAutoPose()) {
                if (pose.looksHardwareReset() && RobotStateStorage.autoPoseMovedFromOrigin()) {
                    return pose.anchor(RobotStateStorage.autoForwardMm, -RobotStateStorage.autoLeftMm,
                            RobotStateStorage.autoHeadingRad);
                }
                return pose.acceptCurrentHardwarePose();
            }
            return pose.anchorManualCorner();
        }
        return poseInitialized || pose.hasPositionAnchor();
    }

    private void showBattery(BatteryVoltage battery) {
        if (battery.low()) {
            telemetry.addData("<font color='#ff4444'>电池低压</font>", "<font color='#ff4444'>%.2f V</font>",
                    battery.volts());
        } else if (battery.measured()) telemetry.addData("电池", "%.2f V", battery.volts());
        else telemetry.addData("电池", "电压读数不可用");
    }

    private void showFlywheelPid(Flywheel wheel) {
        double[] gains = wheel.pidGains();
        telemetry.addData("飞轮 PID P / I / D", "%s  %.3f / %.3f / %.3f",
                wheel.pidSource(), gains[0], gains[1], gains[2]);
    }

    private static void publishDashboard(FtcDashboard dashboard, BatteryVoltage battery, Turret turret, Flywheel wheel,
                                         boolean feeding, boolean rpmLock, double loopHz) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("loop_hz", loopHz);
        packet.put("battery_V", battery.volts());
        packet.put("feeding", feeding ? 1 : 0);
        packet.put("rpm_lock", rpmLock ? 1 : 0);
        if (turret != null) {
            packet.put("turret_angle_deg", Math.toDegrees(turret.angle));
            packet.put("turret_target_deg", Math.toDegrees(turret.target));
            packet.put("turret_rate_deg_s", Math.toDegrees(turret.speed));
            packet.put("turret_voltage", turret.output() * BatteryVoltage.NOMINAL_VOLTS);
            packet.put("turret_duty", turret.actualDuty());
            packet.put("turret_brake_voltage", turret.brakeDuty() * BatteryVoltage.NOMINAL_VOLTS);
            packet.put("turret_observer_bias", turret.observerBias());
        }
        if (wheel != null && wheel.rpmCalibrated()) {
            packet.put("flywheel_rpm", wheel.rpm());
            packet.put("flywheel_target_rpm", wheel.target * 60 / wheel.ticksPerRev);
            packet.put("flywheel_voltage", wheel.deliveredVolts());
            packet.put("flywheel_pid_integral_voltage", wheel.integralVolts());
            packet.put("flywheel_boosting", wheel.boosting() ? 1 : 0);
        }
        dashboard.sendTelemetryPacket(packet);
    }

    private static void logPose(PinpointPose pose, Turret turret, boolean idle, boolean rt, boolean firing) {
        RobotLog.ii("BioBuzzPose",
                "RT=%b SB=%b idle=%b status=%s rawYaw=%+.2f gyro=%+.2f controlYaw=%+.2f "
                        + "pods=%d/%d xy=%.1f/%.1f turret=%+.2f target=%+.2f turretValid=%b",
                rt, firing, idle, pose.status, Math.toDegrees(pose.observedYaw()),
                Math.toDegrees(pose.observedYawRate()), Math.toDegrees(pose.heading), pose.encoderX(), pose.encoderY(),
                pose.forward, pose.left, turret == null ? Double.NaN : Math.toDegrees(turret.angle),
                turret == null ? Double.NaN : Math.toDegrees(turret.target), turret != null && turret.encoderReady());
    }

    private void showTurretCalibration(Turret turret) {
        double[] gains = turret.pidGains();
        double[] coupling = turret.coupling();
        String pidSource = turret.usesSavedPid() ? "02 / 05 实测" : turret.plantIdentified() ? "02 基线" : "默认";
        telemetry.addData("云台 PID P / I / D", "%s  %.3f / %.3f / %.3f", pidSource, gains[0], gains[1], gains[2]);
        telemetry.addData("云台刹车模型 / 摩擦", "%s / %s", turret.plantIdentified() ? "已加载 02" : "默认",
                turret.hasFrictionMap() ? "已加载分段表" : "方向基线");
        telemetry.addData("云台耦合 左 / 右", "%s / %s", couplingStatus(coupling, 0), couplingStatus(coupling, 2));
    }

    private static String couplingStatus(double[] values, int side) {
        return values[side] > 0 ? String.format("%.3f, %.3f", values[side], values[side + 1]) : "模型基线";
    }

    private static RearStatusLed.State rearState(boolean fault, boolean reverse, boolean noFire,
                                                 boolean turretUnlocked, boolean turretLockedZero,
                                                 boolean intake, boolean aimed, boolean rpmLockEnabled) {
        if (fault) return RearStatusLed.State.FAULT;
        if (reverse) return RearStatusLed.State.REVERSE;
        if (noFire) return RearStatusLed.State.NO_FIRE;
        if (turretUnlocked) return RearStatusLed.State.UNLOCKED;
        if (turretLockedZero) return RearStatusLed.State.LOCKED_ZERO;
        if (intake) return RearStatusLed.State.INTAKE;
        if (!aimed) return RearStatusLed.State.DRIVE;
        return rpmLockEnabled ? RearStatusLed.State.AIMED_LOCKED : RearStatusLed.State.AIMED;
    }

    private <T> T optional(String name, Supplier<T> factory) {
        try {
            return factory.get();
        } catch (RuntimeException unavailable) {
            telemetry.log().add("%s不可用：%s", name, unavailable.getMessage());
            return null;
        }
    }
}
