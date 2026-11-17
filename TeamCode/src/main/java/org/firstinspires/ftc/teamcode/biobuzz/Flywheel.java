package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Flywheel {
    public static final double[] DEFAULT_GAINS = {0.40, 0.26, 0.012};
    private final DcMotorEx m7, m8;
    private final Servo blocker;
    private final double[] model;
    public final double ticksPerRev;
    private Pid pid;
    private int previousTicks;
    private double previousVelocity, lastMotion;
    private boolean fault;
    public double speed, target, output, stableSeconds;

    public Flywheel(HardwareMap map, CalibrationStore store) {
        m7 = map.get(DcMotorEx.class, RobotConfig.FLYWHEEL_LEFT);
        m8 = map.get(DcMotorEx.class, RobotConfig.FLYWHEEL_ENCODER_MOTOR);
        configure(m7, RobotConfig.FLYWHEEL_LEFT_DIRECTION);
        configure(m8, RobotConfig.FLYWHEEL_ENCODER_DIRECTION);
        blocker = map.tryGet(Servo.class, RobotConfig.BLOCKER_SERVO);
        feed(false);
        previousTicks = ticks();
        double[] fullModel = store.read(CalibrationStore.FLYWHEEL_FULL_POWER_MODEL, 0, 0, 0, 0);
        model = fullModel[3] >= 40 && fullModel[0] >= 20 && fullModel[2] > 0
                ? fullModel : store.read(CalibrationStore.FLYWHEEL_MODEL, 0, 0, 0);
        ticksPerRev = store.read(CalibrationStore.FLYWHEEL_SCALE, RobotConfig.FLYWHEEL_TICKS_PER_WHEEL_REV)[0];
        setGains(store.read(CalibrationStore.FLYWHEEL_PID, DEFAULT_GAINS));
    }

    private void configure(DcMotorEx motor, DcMotorSimple.Direction direction) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(direction);
    }

    public int ticks() { return m8.getCurrentPosition(); }
    public double reference() { return model[0]; }
    public double fullPowerSpeed() { return model.length == 4 ? model[3] : 0; }
    public double estimatedPowerForSpeed(double speed) { return model[1] + model[2] * speed / reference(); }
    public double estimatedSpeedAtPower(double power) {
        return calibrated() ? reference() * Math.max(0, power - model[1]) / model[2] : 0;
    }
    public boolean calibrated() { return reference() >= 20 && model[2] > 0; }
    public boolean rpmCalibrated() { return calibrated() && ticksPerRev > 0; }
    public boolean faulted() { return fault; }
    public boolean ready() { return !fault && target > 0 && stableSeconds >= 0.25; }
    public boolean hasBlocker() { return blocker != null; }
    public void setGains(double[] gains) { pid = new Pid(gains); }

    public void sample(double dt) {
        try {
            int tick = ticks();
            int delta = tick - previousTicks;
            if (delta != 0) lastMotion = Maths.time();
            previousTicks = tick;
            previousVelocity = speed;
            speed += dt / (0.075 + dt) * (Math.abs(delta / dt) - speed);
            if (output > 0.22 && Maths.time() - lastMotion > 0.85) fault = true;
        } catch (RuntimeException failure) {
            fault = true;
        }
        if (fault) stop();
    }

    public void requestRpm(double rpm, double dt) {
        if (!rpmCalibrated()) {
            stop();
            return;
        }
        regulate(rpm * ticksPerRev / 60, dt);
    }

    public void regulate(double ticksPerSecond, double dt) {
        if (fault || !calibrated() || ticksPerSecond <= 0) {
            stop();
            return;
        }
        if (Math.abs(target - ticksPerSecond) > target * RobotConfig.FLYWHEEL_SPEED_TOLERANCE * 0.5) {
            stableSeconds = 0;
        }
        target = ticksPerSecond;
        double error = (target - speed) / reference();
        double acceleration = (speed - previousVelocity) / (dt * reference());
        double ff = estimatedPowerForSpeed(target);
        double min = Math.max(0, output - 4.0 * dt);
        double max = Math.min(RobotConfig.FLYWHEEL_MAX_POWER, output + RobotConfig.FLYWHEEL_RAMP * dt);
        power(pid.update(error, acceleration, ff, dt, min, max), dt);
        stableSeconds = Math.abs(target - speed) <= target * RobotConfig.FLYWHEEL_SPEED_TOLERANCE
                ? stableSeconds + dt : 0;
    }

    public void power(double desired, double dt) {
        double next = fault ? 0 : Maths.clip(desired, 0, RobotConfig.FLYWHEEL_MAX_POWER);
        next = Math.min(next, output + RobotConfig.FLYWHEEL_RAMP * dt);
        if (output <= 0.22 && next > 0.22) lastMotion = Maths.time();
        output = next;
        m7.setPower(next);
        m8.setPower(next);
    }

    public boolean feed(boolean fire) {
        boolean released = blocker != null && fire && ready();
        if (blocker != null) blocker.setPosition(released ? RobotConfig.SB_FIRE : RobotConfig.SB_IDLE);
        return released;
    }

    public void stop() {
        feed(false);
        m7.setPower(0);
        m8.setPower(0);
        target = output = stableSeconds = 0;
        pid.reset();
    }
}
