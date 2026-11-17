package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.function.Supplier;

@TeleOp(name = "BIOBUZZ 360 云台 + 无头", group = "BIOBUZZ")
public final class BioBuzzTeleOp extends LinearOpMode {
    private double startRight = RobotConfig.CENTER_FROM_LEFT_EDGE_MM / RobotConfig.TILE_MM;
    private double startForward = RobotConfig.CENTER_FROM_REAR_EDGE_MM / RobotConfig.TILE_MM;
    private double startHeading;
    private double driverYawZero;
    private boolean useGoals = true, lastA, lastX, lastY;
    private boolean up, down, left, right, lb, rb;

    @Override public void runOpMode() throws InterruptedException {
        CalibrationStore store = new CalibrationStore(hardwareMap.appContext);
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
                telemetry.addData("云台", turret == null || !turret.calibrated() ? "先运行 02 自测"
                        : store.has(CalibrationStore.TURRET_PLANT) ? "已通过快速辨识" : "默认保守参数，建议跑 02");
                telemetry.addData("飞轮", shooter != null && shooter.rpmCalibrated()
                        ? String.format("START 后怠速 %.0f RPM", RobotConfig.FLYWHEEL_IDLE_RPM)
                        : "先运行 04，自测 RPM 比例和转速曲线");
                telemetry.update();
                sleep(25);
            }
            if (isStopRequested()) return;
            if (turret != null) {
                turret.teachingMode(false);
                turret.homeAtRight();
            }
            lastX = gamepad1.x;
            lastA = gamepad1.a;
            lastY = gamepad1.y;
            pose.update();
            anchor(pose);
            lights.show(RearStatusLed.State.DRIVE);
            boolean movedBeforeAnchor = false;
            double previous = Maths.time();
            while (opModeIsActive()) {
                double timestamp = Maths.time();
                double dt = Maths.clip(timestamp - previous, 0.005, 0.25);
                previous = timestamp;
                pose.update();
                // Re-anchor only when the operator has returned the robot to the entered pose.
                if (gamepad1.y && !lastY && pose.headingValid) {
                    anchor(pose);
                    driverYawZero = 0;
                    movedBeforeAnchor = false;
                    if (turret != null) turret.stop();
                }
                lastY = gamepad1.y;
                if (gamepad1.x && !lastX) {
                    useGoals = !useGoals;
                    if (turret != null) turret.stop();
                }
                lastX = gamepad1.x;

                double f = -gamepad1.left_stick_y;
                double l = -gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x;
                if (!pose.hasPositionAnchor() && !movedBeforeAnchor) {
                    if (!pose.requiresReanchor() && Math.max(Math.hypot(f, l), Math.abs(turn)) < 0.05) anchor(pose);
                    else movedBeforeAnchor = true;
                }
                boolean field = pose.headingUsable();
                if (gamepad1.a && !lastA && field && pose.headingValid) driverYawZero = pose.heading;
                lastA = gamepad1.a;
                if (turret != null) {
                    if (!turret.isHomed()) turret.homeAtRight();
                    turret.sample(dt);
                }
                double effective = 0;
                if (drive != null) effective = field ? drive.field(f, l, turn, pose.heading - driverYawZero)
                        : drive.robot(f, l, turn);
                boolean reverseRequested = gamepad1.right_bumper;
                boolean triggerHeld = !reverseRequested && gamepad1.right_trigger > 0.10f;
                boolean noFireZone = pose.hasPositionAnchor() && FieldTargets.inNoFireZone(pose.forward, pose.left);
                boolean shotRequested = triggerHeld && pose.positionValid && !noFireZone;
                boolean automatic = useGoals || triggerHeld || noFireZone;
                boolean positionForAim = pose.positionValid || pose.recentlyLocated(0.40);
                FieldTargets aim = automatic && field && positionForAim
                        ? FieldTargets.fromPose(pose.forward, pose.left, pose.heading) : null;

                if (turret != null) turret.aim(pose, aim, effective, dt);
                if (!shotRequested && shooter != null) shooter.feed(false);
                if (intake != null) intake.set(gamepad1.left_bumper && !triggerHeld, shotRequested, reverseRequested);
                shot.updateChannel(shotRequested && intake != null, System.nanoTime());
                if (reverseRequested && intake != null) telemetry.addData("供弹通道", "反转");
                if (noFireZone) telemetry.addData("射击", "禁射区，继续瞄准");
                else if (triggerHeld && !pose.positionValid) telemetry.addData("射击", "等待定位");
                boolean aimed = field && aim != null && turret != null && turret.aimReady();
                boolean firing = false;
                if (shooter != null) {
                    shooter.sample(dt);
                    double rpm = shotRequested && aim != null ? ShooterProfile.targetRpm(aim.goal, aim.distanceMm)
                            : RobotConfig.FLYWHEEL_IDLE_RPM;
                    shooter.requestRpm(rpm, dt);
                    if (shotRequested) {
                        firing = shooter.feed(shot.canOpenBlocker(shooter.ready(), aimed, System.nanoTime()));
                        if (firing) telemetry.addData("射击", "SB 已打开");
                        else if (intake == null) telemetry.addData("射击", "供弹机构不可用");
                        else if (!shooter.hasBlocker()) telemetry.addData("射击", "找不到 SB");
                        else if (!shooter.ready()) telemetry.addData("射击", "等待飞轮复速");
                        else if (!aimed) telemetry.addData("射击", "等待瞄准 ±2.5°");
                        else telemetry.addData("射击", "等待供弹 300 ms");
                    }
                    telemetry.addData("飞轮", shooter.faulted() ? "编码器故障，请停机检查"
                            : !shooter.rpmCalibrated() ? "先运行 04，自测 RPM 比例和转速曲线"
                            : String.format("%.0f / %.0f RPM", shooter.speed * 60 / shooter.ticksPerRev,
                                    shooter.target * 60 / shooter.ticksPerRev));
                }
                boolean mechanismFault = drive == null || intake == null || turret == null || shooter == null
                        || !turret.calibrated() || !turret.encoderReady() || !turret.motorResponding()
                        || shooter.faulted() || !shooter.rpmCalibrated() || !shooter.hasBlocker();
                lights.update(rearState(mechanismFault || !field || !positionForAim, reverseRequested,
                        firing, noFireZone, triggerHeld, gamepad1.left_bumper, aimed));
                telemetry.addData("底盘", field ? "无头" : "车头模式");
                if (turn != 0) telemetry.addData("底盘转向输出", "%+.2f", effective);
                telemetry.addData("右 / 前 / 车头", "%.2f / %.2f 格 / %+.1f°", -pose.left / RobotConfig.TILE_MM,
                        pose.forward / RobotConfig.TILE_MM, Math.toDegrees(pose.heading));
                telemetry.addData("Pinpoint", pose.status);
                telemetry.addData("目标", aim != null ? String.format("得分点 %d / %+.1f°%s", aim.goal,
                        Math.toDegrees(aim.bearing), pose.positionValid ? "" : "（最近有效位置）")
                        : automatic ? "自动瞄准：等待定位" : "手动锁方向（X 恢复自瞄）");
                if (pose.faults != 0) telemetry.addData("最近一次定位异常", "%s，累计 %d",
                        pose.lastFault, pose.faults);
                if (pose.badReads != 0 || gamepad1.back) {
                    telemetry.addData("Pinpoint 固件 / 坏读 / 补读成功 / 连续坏读", "V%d / %d / %d / %d",
                            pose.firmwareVersion, pose.badReads, pose.recoveredReads, pose.badReadStreak);
                }
                if (gamepad1.back && pose.headingJumps > 0) {
                    telemetry.addData("航向跳变 / 补读成功", "%d / %d，最近 %+.1f° / %.0f ms，传感器 %+.0f°/s",
                            pose.headingJumps, pose.recoveredHeadingJumps, Math.toDegrees(pose.lastJumpAngle),
                            1000 * pose.lastJumpSeconds, Math.toDegrees(pose.lastJumpRate));
                }
                if (!pose.hasPositionAnchor() && (movedBeforeAnchor || pose.requiresReanchor())) {
                    telemetry.addLine("定位需要重新对齐，回起始位按 Y");
                }
                if (!pose.positionValid || gamepad1.back) {
                    telemetry.addData("里程轮原始计数 X / Y", "%d / %d", pose.encoderX(), pose.encoderY());
                    telemetry.addData("Pinpoint 航向 / 无头航向 / 角速度", "%+.1f° / %+.1f° / %+.1f°/s",
                            Math.toDegrees(pose.sensorYaw()), Math.toDegrees(Maths.wrap(pose.heading - driverYawZero)),
                            Math.toDegrees(pose.yawRate));
                }
                if (turret != null) {
                    String status = !turret.calibrated() ? "先运行 02 自测"
                            : !turret.encoderReady() || !turret.motorResponding() ? "读数或电机恢复中"
                            : turret.rewinding() ? "PID 回转"
                            : String.format("发射朝向 %+.1f°", Math.toDegrees(Maths.wrap(pose.heading + turret.boreRelative())));
                    telemetry.addData("云台", status);
                    if (gamepad1.back) {
                        telemetry.addData("云台行程角 / 电机功率 / 刹车", "%+.1f° / %+.2f / %+.2f",
                                Math.toDegrees(turret.angle), turret.output(), turret.brakeDuty());
                    }
                    if (field && turret.calibrated() && turret.encoderReady()) {
                        telemetry.addData("云台朝向误差", "%+.1f°", Math.toDegrees(turret.worldError));
                    }
                }
                telemetry.addLine("LB 吸取 | RT 射击 | RB 反转 | A 无头归零 | X 切换瞄准 | Y 起点重定位");
                telemetry.update();
                sleep(15);
            }
        } finally {
            try {
                if (shooter != null) shooter.stop();
                if (drive != null) drive.stop();
                if (turret != null) turret.teachingMode(false);
                if (intake != null) intake.set(false, false, false);
            } finally {
                lights.stop();
            }
        }
    }

    private static RearStatusLed.State rearState(boolean fault, boolean reverse, boolean firing,
                                                 boolean noFire, boolean trigger, boolean intake, boolean aimed) {
        if (fault) return RearStatusLed.State.FAULT;
        if (reverse) return RearStatusLed.State.REVERSE;
        if (firing) return RearStatusLed.State.FIRING;
        if (noFire) return RearStatusLed.State.NO_FIRE;
        if (trigger) return RearStatusLed.State.SPINUP;
        if (intake) return RearStatusLed.State.INTAKE;
        return aimed ? RearStatusLed.State.AIMED : RearStatusLed.State.DRIVE;
    }

    private void anchor(PinpointPose pose) {
        pose.anchor(startForward * RobotConfig.TILE_MM, startRight * RobotConfig.TILE_MM,
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
