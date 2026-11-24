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
        private final Servo right, left;
        private boolean switched;

        public TaskServos(HardwareMap map) {
            right = map.tryGet(Servo.class, RobotConfig.TASK_RIGHT_SERVO);
            left = map.tryGet(Servo.class, RobotConfig.TASK_LEFT_SERVO);
            if (right != null) right.setPosition(RobotConfig.FR_HOME);
            if (left != null) left.setPosition(RobotConfig.FL_HOME);
        }

        public void toggle() {
            setLowered(!switched);
        }

        public void setLowered(boolean lowered) {
            if (!configured() || switched == lowered) return;
            right.setPosition(lowered ? RobotConfig.FR_OTHER : RobotConfig.FR_HOME);
            left.setPosition(lowered ? RobotConfig.FL_OTHER : RobotConfig.FL_HOME);
            switched = lowered;
        }

        public boolean configured() { return right != null && left != null; }

        public void stop() {
            if (right != null) ControlLoop.safeStop(RobotConfig.TASK_RIGHT_SERVO, () -> right.setPosition(RobotConfig.FR_HOME));
            if (left != null) ControlLoop.safeStop(RobotConfig.TASK_LEFT_SERVO, () -> left.setPosition(RobotConfig.FL_HOME));
            switched = false;
        }

        public String status() {
            if (right == null && left == null) return "未配置 FR、FL";
            if (right == null) return "未配置 FR";
            if (left == null) return "未配置 FL";
            return switched ? "0.00 / 1.00" : "0.94 / 0.06";
        }
    }
}
