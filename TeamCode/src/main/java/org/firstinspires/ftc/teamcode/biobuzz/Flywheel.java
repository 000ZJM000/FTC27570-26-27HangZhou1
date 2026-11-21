package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Flywheel {
    public static final double[] DEFAULT_GAINS = {0.40, 0.26, 0.012};
    private static final double MAX_POWER_UPDATE_SECONDS = 0.05;
    private final DcMotorEx m7, m8;
    private final Servo blocker;
    private final double[] model;
    private final boolean savedPid;
    public final double ticksPerRev;
    private Pid pid;
    private double[] gains;
    private int previousTicks;
    private double lastSampleTime, lastMotion, sampleDt, signedSpeed, acceleration;
    private boolean fault;
    public double speed, target, output;

    public Flywheel(HardwareMap map, CalibrationStore store) {
        m7 = map.get(DcMotorEx.class, RobotConfig.FLYWHEEL_LEFT);
        m8 = map.get(DcMotorEx.class, RobotConfig.FLYWHEEL_ENCODER_MOTOR);
        double[] directions = store.readExact(CalibrationStore.FLYWHEEL_DIRECTIONS, 2);
        boolean validDirections = directions != null && Math.abs(directions[0]) == 1 && Math.abs(directions[1]) == 1;
        configure(m7, validDirections ? directionFromSign(directions[0]) : RobotConfig.FLYWHEEL_LEFT_DIRECTION);
        configure(m8, validDirections ? directionFromSign(directions[1]) : RobotConfig.FLYWHEEL_ENCODER_DIRECTION);
        blocker = map.tryGet(Servo.class, RobotConfig.BLOCKER_SERVO);
        feed(false);
        previousTicks = ticks();
        lastSampleTime = Maths.time();
        double[] fullModel = store.readExact(CalibrationStore.FLYWHEEL_FULL_POWER_MODEL, 6);
        if (fullModel == null) fullModel = store.read(CalibrationStore.FLYWHEEL_FULL_POWER_MODEL, 0, 0, 0, 0);
        model = fullModel[3] >= 40 && fullModel[0] >= 20 && fullModel[2] > 0
                ? fullModel : store.read(CalibrationStore.FLYWHEEL_MODEL, 0, 0, 0);
        ticksPerRev = store.read(CalibrationStore.FLYWHEEL_SCALE, RobotConfig.FLYWHEEL_TICKS_PER_WHEEL_REV)[0];
        double[] stored = store.readExact(CalibrationStore.FLYWHEEL_PID, 3);
        savedPid = stored != null && stored[0] >= 0 && stored[1] >= 0 && stored[2] >= 0
                && (stored[0] > 0 || stored[1] > 0 || stored[2] > 0);
        setGains(savedPid ? stored : DEFAULT_GAINS);
    }

    private static DcMotorSimple.Direction directionFromSign(double sign) {
        return sign > 0 ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
    }

    private void configure(DcMotorEx motor, DcMotorSimple.Direction direction) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(direction);
    }

    public int ticks() { return m8.getCurrentPosition(); }
    public int leftTicks() { return m7.getCurrentPosition(); }
    public DcMotorSimple.Direction leftDirection() { return m7.getDirection(); }
    public DcMotorSimple.Direction encoderDirection() { return m8.getDirection(); }
    public double reference() { return model[0]; }
    public double fullPowerSpeed() { return model.length >= 4 ? model[3] : 0; }
    private boolean hasMeasuredTestRange() { return model.length == 6 && model[4] > 0 && model[5] > 0; }
    public double lowestMeasuredSpeed() { return hasMeasuredTestRange() ? model[4] : 0; }
    public double longestMeasuredTransition() { return hasMeasuredTestRange() ? model[5] : 0; }
    public double estimatedPowerForSpeed(double speed) { return model[1] + model[2] * speed / reference(); }
    public double estimatedSpeedAtPower(double power) {
        return calibrated() ? reference() * Math.max(0, power - model[1]) / model[2] : 0;
    }
    public boolean calibrated() { return reference() >= 20 && model[2] > 0; }
    public boolean rpmCalibrated() { return calibrated() && ticksPerRev > 0; }
    public double maxTargetRpm() {
        return rpmCalibrated() ? 60 * estimatedSpeedAtPower(RobotConfig.FLYWHEEL_TARGET_MAX_DUTY) / ticksPerRev : 0;
    }
    public boolean faulted() { return fault; }
    public boolean readyForShot() {
        return readyForShot(RobotConfig.SHOT_RPM_TOLERANCE);
    }
    boolean readyForShot(double relativeTolerance) {
        return !fault && target > 0 && Math.abs(speed - target) <= target * relativeTolerance;
    }
    public boolean hasBlocker() { return blocker != null; }
    public boolean usesSavedPid() { return savedPid; }
    public double[] pidGains() { return gains.clone(); }
    public void setGains(double[] gains) {
        this.gains = gains.clone();
        pid = new Pid(this.gains);
    }

    public void sample() {
        sampleDt = 0;
        if (Maths.time() - lastSampleTime < 0.010) return;
        try {
            int tick = ticks();
            double now = Maths.time();
            double elapsed = now - lastSampleTime;
            lastSampleTime = now;
            int delta = tick - previousTicks;
            if (delta != 0) lastMotion = now;
            previousTicks = tick;
            double previousSpeed = speed;
            signedSpeed += elapsed / (0.075 + elapsed) * (delta / elapsed - signedSpeed);
            speed = Math.abs(signedSpeed);
            if (elapsed > 0.20) {
                // Use the average speed with a bounded update and keep the integral through a delayed read.
                acceleration = 0;
                sampleDt = MAX_POWER_UPDATE_SECONDS;
            } else {
                // Filter the derivative separately so a one-tick change does not kick the duty.
                double measuredAcceleration = (speed - previousSpeed) / elapsed;
                acceleration += elapsed / (0.15 + elapsed) * (measuredAcceleration - acceleration);
                sampleDt = elapsed;
            }
            if (output > 0.22 && now - lastMotion > 0.85) fault = true;
        } catch (RuntimeException failure) {
            fault = true;
        }
        if (fault) stop();
    }

    public void requestRpm(double rpm) {
        regulate(Maths.clip(rpm, 0, maxTargetRpm()) * ticksPerRev / 60);
    }

    void trackRpm(double rpm) {
        regulate(Maths.clip(rpm, 0, maxTargetRpm()) * ticksPerRev / 60, true);
    }

    public void regulate(double ticksPerSecond) {
        regulate(ticksPerSecond, false);
    }

    private void regulate(double ticksPerSecond, boolean tracking) {
        if (fault || !calibrated() || !Double.isFinite(ticksPerSecond) || ticksPerSecond <= 0) {
            stop();
            return;
        }
        double allowedChange = tracking ? 0.10 * target : 0;
        if (Math.abs(target - ticksPerSecond) > allowedChange) {
            pid.reset();
        }
        target = ticksPerSecond;
        double dt = sampleDt;
        sampleDt = 0;
        if (dt <= 0) return;
        double error = (target - speed) / reference();
        double ff = estimatedPowerForSpeed(target);
        double powerDt = Math.min(dt, MAX_POWER_UPDATE_SECONDS);
        double min = Math.max(0, output - 4.0 * powerDt);
        double max = Math.min(RobotConfig.FLYWHEEL_MAX_POWER, output + RobotConfig.FLYWHEEL_RAMP * powerDt);
        double speedError = Math.abs(target - speed);
        // Limit integral buildup on large, fast steps while still correcting a persistent speed error.
        boolean nearTarget = speedError <= 0.10 * target;
        boolean lowAcceleration = Math.abs(acceleration) <= 0.15 * target;
        // A saved PID can have zero integral gain. Near steady speed, still trim model error in either direction.
        double minimumIntegralGain = nearTarget && lowAcceleration ? DEFAULT_GAINS[1] : 0;
        power(pid.update(error, acceleration / reference(), ff, dt, min, max,
                nearTarget || lowAcceleration, minimumIntegralGain), powerDt);
    }

    double feedforwardDuty() { return target > 0 ? estimatedPowerForSpeed(target) : 0; }
    double integralDuty() { return pid.integralDuty(); }

    public void power(double desired, double dt) {
        double next = fault ? 0 : Maths.clip(desired, 0, RobotConfig.FLYWHEEL_MAX_POWER);
        next = Math.min(next, output + RobotConfig.FLYWHEEL_RAMP * dt);
        if (output <= 0.22 && next > 0.22) lastMotion = Maths.time();
        output = next;
        m7.setPower(next);
        m8.setPower(next);
    }

    public boolean feed(boolean fire) {
        boolean released = blocker != null && fire && !fault && target > 0;
        if (blocker != null) blocker.setPosition(released ? RobotConfig.SB_FIRE : RobotConfig.SB_IDLE);
        return released;
    }

    public void stop() {
        feed(false);
        m7.setPower(0);
        m8.setPower(0);
        target = output = sampleDt = 0;
        pid.reset();
    }
}
