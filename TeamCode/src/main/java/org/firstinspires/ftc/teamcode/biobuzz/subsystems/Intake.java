package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.ControlLoop;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;

/** Intake / feed motor + SL/SR lift servos + auxiliary FR/FL task servos. */
public final class Intake {
    private final DcMotor motor;
    private final Servo sl, sr;
    private boolean lowered;
    private double raisedAt;
    private double power;

    public Intake(HardwareMap map) {
        motor = map.get(DcMotor.class, RobotConfig.INTAKE);
        motor.setPower(0);
        motor.setDirection(RobotConfig.INTAKE_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sl = map.get(Servo.class, RobotConfig.INTAKE_LEFT_SERVO);
        sr = map.get(Servo.class, RobotConfig.INTAKE_RIGHT_SERVO);
        sl.setPosition(RobotConfig.SL_UP);
        sr.setPosition(RobotConfig.SR_UP);
        raisedAt = Maths.time();
    }

    public boolean readyToFeed() {
        return !lowered && power >= 0 && Maths.time() - raisedAt >= RobotConfig.INTAKE_RAISE_SETTLE_SECONDS;
    }

    public void set(boolean intakeRequested, boolean shotRequested, boolean reverseRequested) {
        boolean lower = intakeRequested && !shotRequested && !reverseRequested;
        boolean raising = lowered && !lower;
        if (raising || (!lower && !shotRequested && !reverseRequested)) setPower(0);
        sl.setPosition(lower ? RobotConfig.SL_INTAKE : RobotConfig.SL_UP);
        sr.setPosition(lower ? RobotConfig.SR_INTAKE : RobotConfig.SR_UP);
        if (raising) {
            raisedAt = Maths.time();
            RobotLog.ii("BioBuzzShot", "SL/SR raised");
        }
        lowered = lower;
        if (reverseRequested) setPower(-RobotConfig.INTAKE_REVERSE_POWER);
        else if (shotRequested) {
            boolean starting = power != RobotConfig.FEED_POWER;
            setPower(readyToFeed() ? RobotConfig.FEED_POWER : 0);
            if (starting && power > 0) RobotLog.ii("BioBuzzShot", "m-5 feed at %.2f", power);
        } else if (lowered) setPower(RobotConfig.INTAKE_POWER);
    }

    public void stop() {
        ControlLoop.safeStop(RobotConfig.INTAKE, () -> motor.setPower(0));
        ControlLoop.safeStop(RobotConfig.INTAKE_LEFT_SERVO, () -> sl.setPosition(RobotConfig.SL_UP));
        ControlLoop.safeStop(RobotConfig.INTAKE_RIGHT_SERVO, () -> sr.setPosition(RobotConfig.SR_UP));
        if (lowered) raisedAt = Maths.time();
        lowered = false;
        power = 0;
    }

    private void setPower(double next) {
        motor.setPower(next);
        power = next;
    }

    /** Paired FR / FL auxiliary servos. */
    public static final class TaskServos {
        private enum Position { INIT, HOME, LOWERED }

        private final Servo right, left;
        private Position position = Position.INIT;

        public TaskServos(HardwareMap map) {
            right = map.tryGet(Servo.class, RobotConfig.TASK_RIGHT_SERVO);
            left = map.tryGet(Servo.class, RobotConfig.TASK_LEFT_SERVO);
            if (right != null) right.setPosition(RobotConfig.FR_INIT);
            if (left != null) left.setPosition(RobotConfig.FL_INIT);
        }

        public void moveToHome() {
            if (right != null) right.setPosition(RobotConfig.FR_HOME);
            if (left != null) left.setPosition(RobotConfig.FL_HOME);
            position = Position.HOME;
        }

        public void toggle() {
            setLowered(position != Position.LOWERED);
        }

        public void setLowered(boolean lowered) {
            apply(lowered ? Position.LOWERED : Position.HOME, false);
        }

        public void forceLowered(boolean lowered) {
            apply(lowered ? Position.LOWERED : Position.HOME, true);
        }

        private void apply(Position target, boolean force) {
            if (right == null && left == null) return;
            if (!force && position == target) return;
            boolean lowered = target == Position.LOWERED;
            if (right != null) ControlLoop.safeStop(RobotConfig.TASK_RIGHT_SERVO,
                    () -> right.setPosition(lowered ? RobotConfig.FR_OTHER : RobotConfig.FR_HOME));
            if (left != null) ControlLoop.safeStop(RobotConfig.TASK_LEFT_SERVO,
                    () -> left.setPosition(lowered ? RobotConfig.FL_OTHER : RobotConfig.FL_HOME));
            position = target;
            RobotLog.ii("BioBuzzAuto", "FR/FL %s -> %s", force ? "force" : "set", status());
        }

        public boolean configured() { return right != null && left != null; }

        public void stop() {
            if (position == Position.INIT) return;
            if (right != null) ControlLoop.safeStop(RobotConfig.TASK_RIGHT_SERVO, () -> right.setPosition(RobotConfig.FR_HOME));
            if (left != null) ControlLoop.safeStop(RobotConfig.TASK_LEFT_SERVO, () -> left.setPosition(RobotConfig.FL_HOME));
            position = Position.HOME;
        }

        public String status() {
            if (right == null && left == null) return "未配置 FR、FL";
            if (right == null) return "未配置 FR";
            if (left == null) return "未配置 FL";
            if (position == Position.INIT) return String.format("初始化 %.3f / %.3f", RobotConfig.FR_INIT, RobotConfig.FL_INIT);
            return position == Position.LOWERED
                    ? String.format("放下 %.2f / %.2f", RobotConfig.FR_OTHER, RobotConfig.FL_OTHER)
                    : String.format("收回 %.2f / %.3f", RobotConfig.FR_HOME, RobotConfig.FL_HOME);
        }
    }
}
