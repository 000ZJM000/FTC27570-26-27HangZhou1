package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Drive {
    private final DcMotorEx[] motors = new DcMotorEx[4];

    public Drive(HardwareMap map) {
        for (int i = 0; i < motors.length; i++) {
            motors[i] = map.get(DcMotorEx.class, RobotConfig.DRIVE[i]);
            motors[i].setPower(0);
            motors[i].setDirection(RobotConfig.DRIVE_DIRECTIONS[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public double field(double forward, double left, double turn, double heading) {
        return field(forward, left, turn, heading, 1);
    }

    double field(double forward, double left, double turn, double heading, double maxPower) {
        double c = Math.cos(heading), s = Math.sin(heading);
        return robot(c * forward + s * left, -s * forward + c * left, turn, maxPower);
    }

    // Forward, left, and counterclockwise are positive everywhere above the wheel mixer.
    public double robot(double forward, double left, double turn) {
        return robot(forward, left, turn, 1);
    }

    double robot(double forward, double left, double turn, double maxPower) {
        double right = -left;
        double clockwise = -turn;
        double fl = forward + right + clockwise;
        double bl = forward - right + clockwise;
        double fr = forward - right - clockwise;
        double br = forward + right - clockwise;
        double peak = Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br)));
        double scale = Math.max(1, peak / maxPower);
        motors[0].setPower(fl / scale);
        motors[1].setPower(bl / scale);
        motors[2].setPower(fr / scale);
        motors[3].setPower(br / scale);
        return turn / scale;
    }

    public void stop() { robot(0, 0, 0); }
}
