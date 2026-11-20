package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.function.Supplier;

@TeleOp(name = "BIOBUZZ 360 云台 + 无头", group = "BIOBUZZ")
public class BioBuzzTeleOp extends LinearOpMode {
    private static final double LOOP_PERIOD = 0.015;
    private static final double DISPLAY_PERIOD = 0.25;
    private static final double RUN_SHOT_POWER = 0.50;
    private double startRight = RobotConfig.CENTER_FROM_LEFT_EDGE_MM / RobotConfig.TILE_MM;
    private double startForward = RobotConfig.CENTER_FROM_REAR_EDGE_MM / RobotConfig.TILE_MM;
    private double startHeading;
    private double driverYawZero;
    private boolean useGoals = true, lastA, lastX, lastY;
    private boolean up, down, left, right, lb, rb;

    protected boolean usesShotCalibration() { return false; }

    @Override public void runOpMode() throws InterruptedException {
        CalibrationStore store = new CalibrationStore(hardwareMap.appContext);
        boolean calibratedMode = usesShotCalibration();
        ShotVectorField leftField = calibratedMode ? new ShotVectorField(store, false) : null;
        ShotVectorField rightField = calibratedMode ? new ShotVectorField(store, true) : null;
        Drive drive = optional("底盘", () -> new Drive(hardwareMap));
        Turret turret = optional("云台", () -> new Turret(hardwareMap, store));
        Flywheel shooter = optional("飞轮", () -> new Flywheel(hardwareMap, store));
        Intake intake = optional("吸取", () -> new Intake(hardwareMap));
        PinpointPose pose = new PinpointPose(hardwareMap);
        ShotSequence shot = new ShotSequence();

        Servo pitch = hardwareMap.tryGet(Servo.class, RobotConfig.PITCH_SERVO);
        if (pitch != null) pitch.setPosition(RobotConfig.ST_HOME);
        if (turret != null) turret.teachingMode(true);
        boolean imuStarted = false;
        double imuRetry = 0;
        RearStatusLed lights = RearStatusLed.open(hardwareMap);
        try {
            lights.show(RearStatusLed.State.INIT);
            while (opModeInInit()) {
                if (!imuStarted && Maths.time() >= imuRetry) {
                    imuStarted = pose.startImu();
                    imuRetry = Maths.time() + 1;
                }
                pose.update();
                readStartingPose();
                telemetry.addLine("INIT 时底盘静止，让 Pinpoint 校零。云台摆到最右后 START。");
                telemetry.addLine("方向键每次 0.25 格 | LB/RB 改车头角度 15°（左为正）");
                telemetry.addData("右 / 前 / 车头", "%.2f / %.2f 格 / %+.0f°",
                        startRight, startForward, startHeading);
                telemetry.addData("Pinpoint", pose.status);
                if (!lights.configured()) telemetry.addData("尾灯", "未配置 LED");
                if (turret == null || !turret.calibrated()) telemetry.addData("云台", "先运行 02 自测");
                else showTurretCalibration(turret);
                telemetry.addData("飞轮", shooter != null && shooter.rpmCalibrated()
                        ? calibratedMode ? "从 06 校准读取目标 RPM" : String.format("START 后目标 %.0f RPM",
                                Math.min(RobotConfig.FLYWHEEL_TARGET_RPM, shooter.maxTargetRpm()))
                        : "先运行 04，自测 RPM 比例和转速曲线");
                if (shooter != null) telemetry.addData("飞轮转向 m-7 / m-8", "%s / %s",
                        shooter.leftDirection(), shooter.encoderDirection());
                telemetry.update();
                sleep(25);
            }
            if (isStopRequested()) return;
            if (turret != null) {
                // START confirms the turret is at the right stop. Do not retry from an arbitrary angle.
                turret.teachingMode(false);
                if (turret.calibrated()) turret.homeAtRight();
            }
            lastX = gamepad1.x;
            lastA = gamepad1.a;
            lastY = gamepad1.y;
            left = gamepad1.dpad_left;
            right = gamepad1.dpad_right;
            double maxRpm = shooter != null ? shooter.maxTargetRpm() : 0;
            double targetRpm = Math.min(RobotConfig.FLYWHEEL_TARGET_RPM, maxRpm);
            double rpmTrim = 0;
            if (!imuStarted) imuStarted = pose.startImu();
            pose.update();
            if (isStopRequested()) return;
            anchor(pose);
            lights.show(RearStatusLed.State.DRIVE);
            boolean movedBeforeAnchor = false;
            boolean lastTriggerPressed = false;
            boolean blockerOpen = false;
            boolean noFireZone = false;
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            double previous = Maths.time();
            double nextPoseLog = previous, nextDisplay = previous;
            while (opModeIsActive()) {
                double loopStart = Maths.time();
                boolean triggerPressed = gamepad1.right_trigger > 0.10f;
                boolean triggerChanged = triggerPressed != lastTriggerPressed;
                if (triggerChanged) {
                    RobotLog.ii("BioBuzzShot", "RT %s", triggerPressed ? "pressed" : "released");
                    lastTriggerPressed = triggerPressed;
                }
                double f = Maths.deadband(-gamepad1.left_stick_y);
                double l = Maths.deadband(-gamepad1.left_stick_x);
                double turn = Maths.deadband(-gamepad1.right_stick_x);
                boolean driveIdle = f == 0 && l == 0 && turn == 0;
                pose.update();
                if (isStopRequested()) break;
                // Re-anchor only when the operator has returned the robot to the entered pose.
                if (gamepad1.y && !lastY) {
                    if (!imuStarted) imuStarted = pose.startImu();
                    boolean reanchored = anchor(pose);
                    if (isStopRequested()) break;
                    if (reanchored) {
                        driverYawZero = 0;
                        movedBeforeAnchor = false;
                        if (turret != null) turret.stop();
                    }
                }
                lastY = gamepad1.y;
                if (gamepad1.x && !lastX) {
                    useGoals = !useGoals;
                    if (turret != null) turret.stop();
                }
                lastX = gamepad1.x;

                if (!pose.hasPositionAnchor() && !movedBeforeAnchor) {
                    if (!pose.requiresReanchor() && driveIdle) anchor(pose);
                    else movedBeforeAnchor = true;
                }
                boolean field = pose.headingUsable();
                if (gamepad1.a && !lastA && field && pose.headingValid) driverYawZero = pose.heading;
                lastA = gamepad1.a;
                double timestamp = Maths.time();
                double dt = Maths.clip(timestamp - previous, 0.005, 0.25);
                previous = timestamp;
                if (turret != null) turret.sample(dt);
                boolean reverseRequested = gamepad1.right_bumper;
                boolean triggerHeld = triggerPressed && !reverseRequested;
                double driveLimit = calibratedMode && triggerPressed ? RUN_SHOT_POWER : 1;
                double effective = 0;
                if (drive != null) effective = field ? drive.field(f, l, turn, pose.heading - driverYawZero, driveLimit)
                        : drive.robot(f, l, turn, driveLimit);
                boolean slower = gamepad1.dpad_left && !left;
                boolean faster = gamepad1.dpad_right && !right;
                double rpmStep = slower == faster ? 0 : (faster ? 1 : -1) * RobotConfig.FLYWHEEL_RPM_STEP;
                left = gamepad1.dpad_left;
                right = gamepad1.dpad_right;
                if (pose.positionValid) noFireZone = FieldTargets.inNoFireZone(pose.forward, pose.left);
                boolean automatic = useGoals || noFireZone;
                boolean positionForAim = pose.positionValid || pose.recentlyLocated(0.40);
                double rightTiles = -pose.left / RobotConfig.TILE_MM;
                boolean rightHalf = rightTiles >= ShotVectorField.COLUMNS;
                ShotVectorField.Vector sample = calibratedMode && field && positionForAim
                        ? (rightHalf ? rightField : leftField).sample(rightTiles, pose.forward / RobotConfig.TILE_MM)
                        : null;
                FieldTargets aim = !calibratedMode && automatic && field && positionForAim
                        ? FieldTargets.fromPose(pose.forward, pose.left, pose.heading) : null;

                if (calibratedMode) {
                    double baseRpm = sample == null ? 0 : sample.rpm();
                    targetRpm = Maths.clip(baseRpm + rpmTrim, 0, maxRpm);
                    if (sample == null) targetRpm = 0;
                    else if (rpmStep != 0) {
                        targetRpm = Maths.clip(targetRpm + rpmStep, 0, maxRpm);
                        rpmTrim = targetRpm - baseRpm;
                    }
                } else if (rpmStep != 0) {
                    targetRpm = Maths.clip(targetRpm + rpmStep, 0, maxRpm);
                }
                if (turret != null) {
                    if (calibratedMode) {
                        boolean followSample = automatic && sample != null;
                        turret.aim(pose, followSample ? (rightHalf ? 4 : 3) : 0,
                                followSample ? sample.bearing() : 0, effective, dt);
                    } else turret.aim(pose, aim, effective, dt);
                }
                if (shooter != null) {
                    shooter.sample();
                    shooter.requestRpm(targetRpm);
                }
                boolean calibratedArea = sample != null && pose.positionValid && pose.headingValid;
                boolean angleReady = calibratedMode && turret != null && turret.aimReady();
                boolean shotBlocked = reverseRequested || noFireZone || targetRpm <= 0
                        || intake == null || turret == null || shooter == null
                        || shooter.faulted() || !shooter.rpmCalibrated() || !shooter.hasBlocker()
                        || (calibratedMode && (!calibratedArea || (!shot.awaitingApproval() && !angleReady)));
                boolean checkShot = triggerPressed && !shotBlocked && shot.awaitingApproval();
                boolean turretStill = checkShot && (calibratedMode
                        ? angleReady && (!driveIdle || turret.stableForShot()) : turret.stableForShot());
                boolean speedReady = checkShot && shooter.readyForShot();
                boolean releaseShot = shot.update(triggerPressed, shotBlocked, turretStill, speedReady);

                if (!releaseShot && shooter != null) shooter.feed(false);
                if (intake != null) intake.set(gamepad1.left_bumper && !triggerHeld, releaseShot, reverseRequested, dt);
                boolean firing = releaseShot && shooter.feed(true);
                if (blockerOpen != firing) {
                    RobotLog.ii("BioBuzzShot", "SB %s", firing ? "opened" : "closed");
                    blockerOpen = firing;
                }
                if (triggerChanged || ((triggerPressed || gamepad1.back || driveIdle) && timestamp >= nextPoseLog)) {
                    logPose(pose, turret, driveIdle, triggerPressed, firing);
                    nextPoseLog = timestamp + (triggerPressed || gamepad1.back ? 0.25 : 2);
                }
                if (timestamp >= nextDisplay) {
                    nextDisplay = timestamp + DISPLAY_PERIOD;
                    if (reverseRequested && intake != null) telemetry.addData("供弹通道", "反转");
                    if (noFireZone) telemetry.addData("射击", "禁射区，继续瞄准");
                    else if (triggerHeld) {
                        if (firing) telemetry.addData("射击", "已放行，松开 RT 结束");
                        else if (shot.interrupted()) telemetry.addData("射击", "供弹已中断，松开 RT 后再按");
                        else if (calibratedMode && !calibratedArea) telemetry.addData("射击", "等待有效定位及 06 可达数据");
                        else if (shotBlocked) telemetry.addData("射击", "检查供弹机构、飞轮和目标转速");
                        else if (!turretStill) telemetry.addData("射击", calibratedMode ? "等待云台对准" : "等待云台稳定");
                        else if (!speedReady) telemetry.addData("射击", "等待飞轮转速进入 ±10%");
                    }
                    if (shooter != null) {
                        telemetry.addData("飞轮", shooter.faulted() ? "编码器故障，请停机检查"
                                : !shooter.rpmCalibrated() ? "先运行 04，自测 RPM 比例和转速曲线"
                                : String.format("%.0f / %.0f RPM", shooter.speed * 60 / shooter.ticksPerRev,
                                        shooter.target * 60 / shooter.ticksPerRev));
                        if (gamepad1.back && shooter.rpmCalibrated()) {
                            double[] gains = shooter.pidGains();
                            telemetry.addData("飞轮 PID P / I / D", "%s  %.3f / %.3f / %.3f",
                                    shooter.usesSavedPid() ? "已保存的 04" : "默认", gains[0], gains[1], gains[2]);
                            telemetry.addData("飞轮前馈 / 积分补偿 / 实际功率", "%.3f / %+.3f / %.3f",
                                    shooter.feedforwardDuty(), shooter.integralDuty(), shooter.output);
                            telemetry.addData("飞轮偏差 / 转速上限", "%+.0f / %.0f RPM",
                                    (shooter.speed - shooter.target) * 60 / shooter.ticksPerRev, maxRpm);
                        }
                    }
                    telemetry.addData("底盘", field ? "无头" : "车头模式");
                    if (driveLimit < 1) telemetry.addData("跑打", "底盘输出上限 50%");
                    if (turn != 0) telemetry.addData("底盘转向输出", "%+.2f", effective);
                    telemetry.addData("右 / 前 / 车头", "%.2f / %.2f 格 / %+.1f°", -pose.left / RobotConfig.TILE_MM,
                            pose.forward / RobotConfig.TILE_MM, Math.toDegrees(pose.heading));
                    telemetry.addData("Pinpoint", pose.status);
                    if (!automatic) telemetry.addData("目标", "手动锁方向（X 恢复自瞄）");
                    else if (calibratedMode) telemetry.addData("目标", sample == null ? "06 范围外、不可达或等待定位"
                            : String.format("06 向量 / %+.1f° / %.0f RPM", Math.toDegrees(sample.bearing()), sample.rpm()));
                    else telemetry.addData("目标", aim != null ? String.format("得分点 %d / %+.1f°%s", aim.goal,
                            Math.toDegrees(aim.bearing), pose.positionValid ? "" : "（最近有效位置）")
                            : "自动瞄准：等待定位");
                    if (pose.faults != 0) telemetry.addData("最近一次定位异常", "%s，累计 %d",
                            pose.lastFault, pose.faults);
                    if (pose.badReads != 0 || gamepad1.back) {
                        telemetry.addData("Pinpoint 固件 / 坏读 / 恢复 / 连续坏读", "V%d / %d / %d / %d",
                                pose.firmwareVersion, pose.badReads, pose.recoveredReads, pose.badReadStreak);
                    }
                    if (gamepad1.back && pose.headingJumps > 0) {
                        telemetry.addData("航向跳变 / 恢复", "%d / %d，最近 %+.1f° / %.0f ms，传感器 %+.0f°/s",
                                pose.headingJumps, pose.recoveredHeadingJumps, Math.toDegrees(pose.lastJumpAngle),
                                1000 * pose.lastJumpSeconds, Math.toDegrees(pose.lastJumpRate));
                    }
                    if (gamepad1.back && pose.positionJumps > 0) {
                        telemetry.addData("位置跳变", "%d 次，最近 %.0f mm / %.0f ms",
                                pose.positionJumps, pose.lastPositionJumpMm, 1000 * pose.lastPositionJumpSeconds);
                    }
                    if (!pose.hasPositionAnchor() && (movedBeforeAnchor || pose.requiresReanchor())) {
                        telemetry.addLine("定位需要重新对齐，回起始位按 Y");
                    }
                    if (!pose.positionValid || gamepad1.back) {
                        telemetry.addData("里程轮原始计数 X / Y", "%d / %d", pose.encoderX(), pose.encoderY());
                        telemetry.addData("原始 / 控制 / 无头航向", "%+.2f° / %+.2f° / %+.2f°",
                                Math.toDegrees(pose.observedYaw()), Math.toDegrees(pose.heading),
                                Math.toDegrees(Maths.wrap(pose.heading - driverYawZero)));
                        telemetry.addData("Pinpoint 原始角速度", "%+.2f°/s", Math.toDegrees(pose.observedYawRate()));
                    }
                    if (turret != null) {
                        String status = !turret.calibrated() ? "先运行 02 自测"
                                : !turret.isHomed() ? "零位读取失败，停止并重新 INIT，摆到最右后 START"
                                : !turret.encoderReady() || !turret.motorResponding() ? "读数或电机恢复中"
                                : !field ? "等待 Pinpoint 航向，保持角度"
                                : turret.rewindPredicted() ? "预测回转，等待目标跨过接缝"
                                : turret.rewinding() ? "PID 回转"
                                : !turret.targetReachable() ? "目标在行程之外，等待可达角"
                                : String.format("发射朝向 %+.1f°", Math.toDegrees(Maths.wrap(pose.heading + turret.boreRelative())));
                        telemetry.addData("云台", status);
                        if (turret.rewinding()) {
                            telemetry.addData("回转剩余 / 速度 / 功率", "%.1f° / %+.0f°/s / %+.2f",
                                    Math.toDegrees(turret.target - turret.angle),
                                    Math.toDegrees(turret.speed), turret.output());
                        }
                        if (gamepad1.back) {
                            showTurretCalibration(turret);
                            telemetry.addData("云台目标 / PID 设定 / 行程角 / 速度", "%.1f° / %.1f° / %.1f° / %+.0f°/s",
                                    Math.toDegrees(turret.target), Math.toDegrees(turret.controlTarget()),
                                    Math.toDegrees(turret.angle), Math.toDegrees(turret.speed));
                            telemetry.addData("云台功率 / 刹车 / 右限 / 左限", "%+.2f / %+.2f / %.2f / %.2f",
                                    turret.output(), turret.brakeDuty(), turret.powerLimit(-1), turret.powerLimit(1));
                        }
                        if (field && turret.calibrated() && turret.encoderReady()) {
                            telemetry.addData("云台朝向误差", "%+.1f°", Math.toDegrees(turret.worldError));
                        }
                    }
                    telemetry.addLine("LB 吸取 | RT 射击 | RB 反转 | A 无头归零 | X 切换瞄准 | Y 起点重定位");
                    telemetry.addLine(calibratedMode ? "方向键 左 / 右 微调 06 转速 −100 / +100 RPM"
                            : "方向键 左 / 右 每次 −100 / +100 RPM");
                    telemetry.update();
                }
                // Complete the LED transfer at the end of this loop, away from Pinpoint and the shot.
                if (!triggerPressed && gamepad1.right_trigger <= 0.10f && !isStopRequested()) {
                    boolean mechanismFault = drive == null || intake == null || turret == null || shooter == null
                            || !turret.calibrated() || !turret.encoderReady() || !turret.motorResponding()
                            || shooter.faulted() || !shooter.rpmCalibrated() || !shooter.hasBlocker();
                    boolean aimed = calibratedMode ? automatic && calibratedArea && angleReady
                            : field && aim != null && turret != null && turret.aimReady();
                    lights.update(rearState(mechanismFault || !field || !positionForAim, reverseRequested,
                            noFireZone, gamepad1.left_bumper, aimed));
                }
                double remaining = LOOP_PERIOD - (Maths.time() - loopStart);
                if (remaining > 0) sleep((long) Math.ceil(1000 * remaining));
            }
        } finally {
            try {
                if (shooter != null) shooter.stop();
                if (drive != null) drive.stop();
                if (turret != null) turret.teachingMode(false);
                if (intake != null) intake.set(false, false, false, 0);
            } finally {
                lights.stop();
            }
        }
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
        String pidSource = turret.usesSavedPid() ? "已保存的 05" : turret.plantIdentified() ? "02 基线" : "默认";
        telemetry.addData("云台 PID P / I / D", "%s  %.3f / %.3f / %.3f", pidSource, gains[0], gains[1], gains[2]);
        telemetry.addData("云台刹车模型 / 摩擦", "%s / %s", turret.plantIdentified() ? "已加载 02" : "默认",
                turret.hasFrictionMap() ? "已加载分段表" : "方向基线");
        telemetry.addData("云台耦合 左 / 右", "%s / %s", couplingStatus(coupling, 0), couplingStatus(coupling, 2));
    }

    private static String couplingStatus(double[] values, int side) {
        return values[side] > 0 ? String.format("%.3f, %.3f", values[side], values[side + 1]) : "模型基线";
    }

    private static RearStatusLed.State rearState(boolean fault, boolean reverse, boolean noFire,
                                                 boolean intake, boolean aimed) {
        if (fault) return RearStatusLed.State.FAULT;
        if (reverse) return RearStatusLed.State.REVERSE;
        if (noFire) return RearStatusLed.State.NO_FIRE;
        if (intake) return RearStatusLed.State.INTAKE;
        return aimed ? RearStatusLed.State.AIMED : RearStatusLed.State.DRIVE;
    }

    private boolean anchor(PinpointPose pose) {
        return pose.anchor(startForward * RobotConfig.TILE_MM, startRight * RobotConfig.TILE_MM,
                Math.toRadians(startHeading));
    }

    private void readStartingPose() {
        if (gamepad1.dpad_up && !up) startForward += 0.25;
        if (gamepad1.dpad_down && !down) startForward -= 0.25;
        if (gamepad1.dpad_left && !left) startRight -= 0.25;
        if (gamepad1.dpad_right && !right) startRight += 0.25;
        if (gamepad1.left_bumper && !lb) startHeading += 15;
        if (gamepad1.right_bumper && !rb) startHeading -= 15;
        startRight = Maths.clip(startRight, 0, FieldTargets.FIELD_TILES);
        startForward = Maths.clip(startForward, 0, FieldTargets.FIELD_TILES);
        startHeading = Math.toDegrees(Maths.wrap(Math.toRadians(startHeading)));
        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;
        left = gamepad1.dpad_left;
        right = gamepad1.dpad_right;
        lb = gamepad1.left_bumper;
        rb = gamepad1.right_bumper;
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
