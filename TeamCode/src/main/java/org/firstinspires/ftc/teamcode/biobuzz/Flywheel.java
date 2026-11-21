package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public final class Flywheel {
    private static final double[] DEFAULT_PID = {1.5, 0.35, 0.025};
    private static final double BOOST_ERROR_RPM = 100;
    private static final double COAST_ERROR_RPM = 35;
    private static final double COLD_START_SECONDS = 0.40;
    private static final double OPEN_LOOP_STALL_VOLTS = 4.5;
    private final DcMotorEx m7, m8;
    private final Servo blocker;
    private final BatteryVoltage battery;
    private final String pidSource;
    private double[] gains;
    private Pid pid;
    // [reference ticks/s, kS, kV, kQ] with speed divided by reference and output in volts.
    private final double[] model;
    public final double ticksPerRev;
    private double lastSampleTime = Maths.time(), lastMotion = lastSampleTime;
    private double acceleration, sampleDt, coldStartedAt = Double.NaN;
    private boolean fault, stallWatchdogArmed;
    public double speed, target, output;

    public Flywheel(HardwareMap map, CalibrationStore store) {
        this(map, store, new BatteryVoltage(map));
    }

    public Flywheel(HardwareMap map, CalibrationStore store, BatteryVoltage battery) {
        this.battery = battery;
        m7 = map.get(DcMotorEx.class, RobotConfig.FLYWHEEL_LEFT);
        m8 = map.get(DcMotorEx.class, RobotConfig.FLYWHEEL_ENCODER_MOTOR);
        double[] directions = store.readExact(CalibrationStore.FLYWHEEL_DIRECTIONS, 2);
        boolean valid = directions != null && Math.abs(directions[0]) == 1 && Math.abs(directions[1]) == 1;
        configure(m7, valid ? direction(directions[0]) : RobotConfig.FLYWHEEL_LEFT_DIRECTION);
        configure(m8, valid ? direction(directions[1]) : RobotConfig.FLYWHEEL_ENCODER_DIRECTION);
        blocker = map.tryGet(Servo.class, RobotConfig.BLOCKER_SERVO);
        feed(false);
        double[] curve = store.readExact(CalibrationStore.FLYWHEEL_VOLTAGE_MODEL, 4);
        if (!validModel(curve)) {
            double[] old = store.readExact(CalibrationStore.FLYWHEEL_FULL_POWER_MODEL, 6);
            if (old == null) old = store.readExact(CalibrationStore.FLYWHEEL_FULL_POWER_MODEL, 4);
            if (old == null || !(old[0] >= 20 && old[2] > 0)) {
                old = store.read(CalibrationStore.FLYWHEEL_MODEL, 0, 0, 0);
            }
            curve = new double[]{old[0], 12 * old[1], 12 * old[2], 0};
        }
        model = validModel(curve) ? curve : null;
        ticksPerRev = store.read(CalibrationStore.FLYWHEEL_SCALE, RobotConfig.FLYWHEEL_TICKS_PER_WHEEL_REV)[0];
        double[] calibratedGains = store.readExact(CalibrationStore.FLYWHEEL_VOLTAGE_PID, 3);
        double[] legacyGains = store.readExact(CalibrationStore.FLYWHEEL_PID, 3);
        if (validPid(calibratedGains)) {
            pidSource = store.flywheelPidStale() ? "12V PID 待复测" : "04 实测 12V PID";
            setPidGains(calibratedGains);
        } else if (validPid(legacyGains)) {
            pidSource = "沿用旧版 PID";
            setPidGains(legacyGains);
        } else {
            pidSource = "默认 PID";
            setPidGains(DEFAULT_PID);
        }
    }

    public static boolean validModel(double[] curve) {
        return curve != null && curve.length == 4 && Maths.finite(curve)
                && curve[0] >= 20 && curve[1] >= 0 && curve[1] < 6
                && curve[2] > 0 && curve[3] >= 0
                && curve[1] + curve[2] + curve[3] <= 16;
    }

    private static Direction direction(double sign) { return sign > 0 ? Direction.FORWARD : Direction.REVERSE; }

    private static void configure(DcMotorEx motor, Direction direction) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(direction);
    }

    public int ticks() { return m8.getCurrentPosition(); }
    public int leftTicks() { return m7.getCurrentPosition(); }
    public Direction leftDirection() { return m7.getDirection(); }
    public Direction encoderDirection() { return m8.getDirection(); }
    public boolean rpmCalibrated() { return model != null && ticksPerRev > 0; }
    public boolean faulted() { return fault; }
    public String pidSource() { return pidSource; }
    public double[] pidGains() { return gains.clone(); }
    public double integralVolts() { return 12 * pid.integralDuty(); }

    public void setPidGains(double[] gains) {
        if (!validPid(gains)) throw new IllegalArgumentException("Invalid flywheel PID");
        this.gains = gains.clone();
        pid = new Pid(this.gains);
    }

    private static boolean validPid(double[] gains) {
        return gains != null && gains.length == 3 && Maths.finite(gains)
                && gains[0] >= 0 && gains[0] <= 4.5 && gains[1] >= 0 && gains[1] <= 4
                && gains[2] >= 0 && gains[2] <= 0.15
                && (gains[0] > 0 || gains[1] > 0 || gains[2] > 0);
    }
    public double rpm() { return ticksPerRev > 0 ? 60 * speed / ticksPerRev : 0; }
    public double deliveredVolts() { return battery.nominal(output) * BatteryVoltage.NOMINAL_VOLTS; }
    public double feedforwardVolts() { return model == null || target <= 0 ? 0 : voltsForSpeed(target); }

    private double voltsForSpeed(double ticksPerSecond) {
        double rate = ticksPerSecond / model[0];
        return model[1] + rate * (model[2] + rate * model[3]);
    }

    public double maxTargetRpm() {
        return maxRpmAtVolts(12 * RobotConfig.FLYWHEEL_TARGET_MAX_DUTY);
    }

    public double maxTargetRpmAtBattery() {
        return maxRpmAtVolts(Math.min(12, battery.volts()) * RobotConfig.FLYWHEEL_TARGET_MAX_DUTY);
    }

    private double maxRpmAtVolts(double volts) {
        if (!rpmCalibrated()) return 0;
        double v = Math.max(0, volts - model[1]);
        return 60 * model[0] / ticksPerRev * (2 * v / (model[2] + Math.sqrt(model[2] * model[2] + 4 * model[3] * v)));
    }

    public boolean readyForShot() {
        return !fault && target > 0 && Math.abs(speed - target) <= target * RobotConfig.SHOT_RPM_TOLERANCE;
    }

    boolean readyForShot(boolean feeding) {
        if (fault || target <= 0) return false;
        return feeding ? speed >= 0.82 * target : speed >= 0.92 * target && speed <= 1.12 * target;
    }

    public boolean hasBlocker() { return blocker != null; }

    public void sample() {
        if (fault) return;
        double now = Maths.time();
        battery.update(now);
        sampleDt = 0;
        double elapsed = now - lastSampleTime;
        if (elapsed < 0.010) return;
        lastSampleTime = now;
        try {
            double reading = Math.abs(m8.getVelocity());
            if (!Double.isFinite(reading)) throw new IllegalStateException("m-8 velocity is invalid");
            if (reading > 2) lastMotion = now;
            double prior = speed;
            speed += elapsed / (0.012 + elapsed) * (reading - speed);
            acceleration = elapsed > 0.10 ? 0 : (speed - prior) / elapsed;
            sampleDt = elapsed;
            if (stallWatchdogArmed && now - lastMotion > 0.85) {
                fault = true;
                RobotLog.ww("BioBuzzFlywheel", "No encoder motion at %.2f V, battery %.2f V, target %.0f RPM",
                        deliveredVolts(), battery.volts(), ticksPerRev > 0 ? target * 60 / ticksPerRev : 0);
            }
        } catch (RuntimeException unavailable) {
            fault = true;
            RobotLog.ww("BioBuzzFlywheel", "%s", unavailable.toString());
        }
        if (fault) stop();
    }

    public void requestRpm(double rpm) {
        if (fault || !rpmCalibrated() || !Double.isFinite(rpm) || rpm <= 0) {
            if (target > 0 || output > 0) stop();
            return;
        }
        boolean starting = target <= 0 && rpm() < 200;
        double nextTarget = Maths.clip(rpm, 0, maxTargetRpm()) * ticksPerRev / 60;
        if (target <= 0 || Math.abs(nextTarget - target) > ticksPerRev * 100 / 60) pid.reset();
        target = nextTarget;
        if (starting) coldStartedAt = Maths.time();
        if (sampleDt <= 0) return;
        double dt = Math.min(sampleDt, 0.05);
        sampleDt = 0;
        double errorRpm = 60 * (target - speed) / ticksPerRev;
        // All PID terms are fractions of 12 V. Apply the real battery limit to its anti-windup.
        double available = Math.min(1, battery.nominal(RobotConfig.FLYWHEEL_MAX_POWER));
        if (Double.isFinite(coldStartedAt)) {
            double progress = (Maths.time() - coldStartedAt) / COLD_START_SECONDS;
            if (progress >= 1 || speed >= 0.90 * target) coldStartedAt = Double.NaN;
            else available = Math.min(available, progress);
        }
        double command;
        if (errorRpm > BOOST_ERROR_RPM) command = available;
        else if (errorRpm < -COAST_ERROR_RPM) command = 0;
        else command = pid.update((target - speed) / model[0],
                Maths.clip(acceleration / model[0], -1.5, 1.5), feedforwardVolts() / 12, dt, 0, available);
        writeVolts(12 * command);
    }

    public void powerVolts(double volts, double dt) {
        double limited = Math.min(volts, deliveredVolts() + 12 * RobotConfig.FLYWHEEL_RAMP * dt);
        writeVolts(limited);
    }

    private void writeVolts(double volts) {
        double next = fault ? 0 : battery.duty(Maths.clip(volts, 0, 12) / 12, RobotConfig.FLYWHEEL_MAX_POWER);
        // The lowest open-loop scan voltage may be below breakaway. Start timing at usable power.
        boolean armWatchdog = next > 0.22 && (target > 0 || next * battery.volts() >= OPEN_LOOP_STALL_VOLTS - 1e-6);
        if (armWatchdog && !stallWatchdogArmed) lastMotion = Maths.time();
        stallWatchdogArmed = armWatchdog;
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
        HardwareStop.attempt(RobotConfig.BLOCKER_SERVO, () -> feed(false));
        HardwareStop.attempt(RobotConfig.FLYWHEEL_LEFT, () -> m7.setPower(0));
        HardwareStop.attempt(RobotConfig.FLYWHEEL_ENCODER_MOTOR, () -> m8.setPower(0));
        target = output = sampleDt = 0;
        stallWatchdogArmed = false;
        coldStartedAt = Double.NaN;
        pid.reset();
    }
}
