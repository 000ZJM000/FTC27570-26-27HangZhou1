package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Intake {
    private final DcMotor motor;
    private final Servo sl, sr;

    public Intake(HardwareMap map) {
        motor = map.get(DcMotor.class, RobotConfig.INTAKE);
        motor.setDirection(RobotConfig.INTAKE_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sl = map.get(Servo.class, RobotConfig.INTAKE_LEFT_SERVO);
        sr = map.get(Servo.class, RobotConfig.INTAKE_RIGHT_SERVO);
        set(false, false, false);
    }

    public void set(boolean intakeRequested, boolean shotRequested, boolean reverseRequested) {
        boolean lowered = intakeRequested && !shotRequested && !reverseRequested;
        if (!lowered && !shotRequested && !reverseRequested) motor.setPower(0);
        sl.setPosition(lowered ? RobotConfig.SL_INTAKE : RobotConfig.SL_UP);
        sr.setPosition(lowered ? RobotConfig.SR_INTAKE : RobotConfig.SR_UP);
        if (reverseRequested) motor.setPower(-RobotConfig.INTAKE_REVERSE_POWER);
        else if (shotRequested) motor.setPower(RobotConfig.FEED_POWER);
        else if (lowered) motor.setPower(RobotConfig.INTAKE_POWER);
    }
}
