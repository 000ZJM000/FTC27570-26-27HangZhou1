package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Drive {
    private static final double POWER_CHANGE_PER_SECOND = 2 / 0.035;
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final double[] applied = new double[4];
    private double lastCommand = Maths.time();

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
        double now = Maths.time();
        double step = POWER_CHANGE_PER_SECOND * (now - lastCommand);
        lastCommand = now;
        apply(0, fl / scale, step, maxPower);
        apply(1, bl / scale, step, maxPower);
        apply(2, fr / scale, step, maxPower);
        apply(3, br / scale, step, maxPower);
        return (applied[2] + applied[3] - applied[0] - applied[1]) / 4;
    }

    private void apply(int index, double requested, double step, double limit) {
        double previous = applied[index];
        double next = previous + Maths.clip(requested - previous, -step, step);
        // Reduce immediately to the new cap when battery protection activates.
        next = Maths.clip(next, -limit, limit);
        motors[index].setPower(next);
        applied[index] = next;
    }

    public void stop() {
        lastCommand = Maths.time();
        for (int i = 0; i < motors.length; i++) {
            DcMotorEx motor = motors[i];
            HardwareStop.attempt(RobotConfig.DRIVE[i], () -> motor.setPower(0));
            applied[i] = 0;
        }
    }
}
