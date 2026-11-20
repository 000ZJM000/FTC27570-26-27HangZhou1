package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public final class Intake {
    private static final double MAX_FEED_STEP_SECONDS = 0.04;
    private final DcMotor motor;
    private final Servo sl, sr;
    private boolean lowered;
    private double raisedAt;
    private double power;

    public Intake(HardwareMap map) {
        motor = map.get(DcMotor.class, RobotConfig.INTAKE);
        motor.setDirection(RobotConfig.INTAKE_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sl = map.get(Servo.class, RobotConfig.INTAKE_LEFT_SERVO);
        sr = map.get(Servo.class, RobotConfig.INTAKE_RIGHT_SERVO);
        motor.setPower(0);
        sl.setPosition(RobotConfig.SL_UP);
        sr.setPosition(RobotConfig.SR_UP);
        raisedAt = Maths.time();
    }

    public void set(boolean intakeRequested, boolean shotRequested, boolean reverseRequested, double dt) {
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
        else if (shotRequested) rampFeed(dt);
        else if (lowered) setPower(RobotConfig.INTAKE_POWER);
    }

    private void rampFeed(double dt) {
        if (power < 0 || Maths.time() - raisedAt < RobotConfig.INTAKE_RAISE_SETTLE_SECONDS) {
            setPower(0);
            return;
        }
        double before = power;
        double step = RobotConfig.FEED_POWER_PER_SECOND * Maths.clip(dt, 0, MAX_FEED_STEP_SECONDS);
        setPower(Math.min(RobotConfig.FEED_POWER, before + step));
        if (before == 0 && power > 0) RobotLog.ii("BioBuzzShot", "m-5 ramp starts at %.2f", power);
        if (before < RobotConfig.FEED_POWER && power == RobotConfig.FEED_POWER) {
            RobotLog.ii("BioBuzzShot", "m-5 reached %.2f", power);
        }
    }

    private void setPower(double next) {
        motor.setPower(next);
        power = next;
    }
}
