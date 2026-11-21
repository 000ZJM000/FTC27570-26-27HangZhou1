package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

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
        HardwareStop.attempt(RobotConfig.INTAKE, () -> motor.setPower(0));
        HardwareStop.attempt(RobotConfig.INTAKE_LEFT_SERVO, () -> sl.setPosition(RobotConfig.SL_UP));
        HardwareStop.attempt(RobotConfig.INTAKE_RIGHT_SERVO, () -> sr.setPosition(RobotConfig.SR_UP));
        if (lowered) raisedAt = Maths.time();
        lowered = false;
        power = 0;
    }

    private void setPower(double next) {
        motor.setPower(next);
        power = next;
    }
}
