package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

final class PedroAutoMechanisms {
    private final Telemetry telemetry;
    private final PinpointPose pose;
    private final TaskServos front;
    private final Intake intake;
    private final Flywheel flywheel;
    private final Turret turret;
    private final CalibratedShotMap shotMap;
    private final ShotSequence shot;
    private final double maxRpm;
    private boolean armed, collecting, feeding;
    private double lastUpdate;
    private CornerAim aim;
    private String shotStatus = "等待 START";

    PedroAutoMechanisms(HardwareMap map, Telemetry telemetry, BatteryVoltage battery, PinpointPose pose) {
        this.telemetry = telemetry;
        this.pose = pose;
        CalibrationStore store = new CalibrationStore(map.appContext);
        front = optional("FR/FL", () -> new TaskServos(map));
        intake = optional("吸取", () -> new Intake(map));
        flywheel = optional("飞轮", () -> new Flywheel(map, store, battery));
        turret = optional("云台", () -> new Turret(map, store, battery));
        shotMap = optional("射界数据", () -> new CalibratedShotMap(store));
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
        if (turret != null) {
            turret.teachingMode(false);
            if (turret.calibrated()) turret.homeAtRight();
        }
        armed = true;
        lastUpdate = Maths.time();
    }

    String frontStatus() { return front == null ? "初始化失败" : front.status(); }
    String shotMapStatus() { return shotMap == null ? "射界数据读取失败" : shotMap.description(); }

    void lowerFront(boolean lowered) { if (front != null) front.setLowered(lowered); }

    void collect(boolean enable) {
        if (collecting == enable) return;
        setFeeding(false);
        collecting = enable;
        if (intake != null) intake.set(collecting, false, false);
    }

    boolean update(double now, boolean braking, double turn, boolean triggerPressed) {
        if (!armed) return false;
        double dt = Maths.clip(now - lastUpdate, 0.005, 0.25);
        lastUpdate = now;
        if (turret != null) turret.sample(dt);
        aim = CornerAim.fromPose(shotMap, pose, braking);
        if (turret != null) CornerAim.track(turret, pose, aim, 0, turn, dt);
        double targetRpm = Maths.clip(CornerAim.targetRpm(aim), 0, maxRpm);
        boolean noFireZone = CornerAim.noFireZone(pose);
        boolean trackingTurret = pose.headingUsable() && CornerAim.positionForAim(pose);
        // Each shooting stop holds the same trigger as corner TeleOp in its default direct-feed mode.
        setFeeding(shot.update(triggerPressed, false, noFireZone, trackingTurret, targetRpm, false));
        if (triggerPressed) {
            shotStatus = noFireZone ? "禁射区，继续瞄准"
                    : feeding ? "正在供弹" : shot.waitingStatus();
        }
        return feeding;
    }

    void showVolley(String label, AutoVolley volley) {
        telemetry.addData("自动射击", label);
        telemetry.addData("供弹", "%.2f / %.2f 秒，%s", volley.feedSeconds(), AutoVolley.FEED_SECONDS,
                volley.finished() ? "本轮完成" : shotStatus);
        telemetry.addData("Pinpoint 右 / 前 / 车头", "%.2f / %.2f 格 / %+.1f°，%s",
                -pose.left / RobotConfig.TILE_MM, pose.forward / RobotConfig.TILE_MM,
                Math.toDegrees(pose.heading), pose.status);
        telemetry.addData("射界目标场地方向", aim == null ? "保朝向 / 默认转速"
                : String.format("%+.1f°", Math.toDegrees(aim.bearing)));
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
        if (turret != null) HardwareStop.attempt("云台", () -> turret.teachingMode(false));
        if (intake != null) HardwareStop.attempt("供弹", () -> intake.set(collecting, false, false));
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
