package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.localization.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TurretModels.Brake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TurretModels.Friction;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TurretModels.Observer;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TurretModels.Plant;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.TurretModels.Range;

/** 360-degree turret controller with 12 V compensation, friction/brake feedforward, and forward-zero lock. */
public final class Turret {
    private static final double OUTPUT_LIMIT = RobotConfig.TURRET_MAX_POWER;
    private final DcMotorEx motor, encoder;
    private final BatteryVoltage battery;
    private final double[] geometry;
    private Friction friction;
    private Plant plant;
    private double[] coupling;
    private final Range range;
    private final boolean savedPid;
    private final double shotAimTolerance;
    private final Observer observer = new Observer();
    private Maths.Pid pid;
    private double[] gains;
    private double appliedPower, actualDuty, brakeDuty, controlTarget;
    private boolean holding;
    private int homeTick, lastTick, trackedGoal;
    private Integer initTick;
    private boolean homed, encoderFresh, fieldTracking, motorResponding = true;
    private double encoderAge, lastPower;
    private double stillSeconds;
    private double previousCoupledRate, coupledAcceleration, filteredStick;
    private double worldBearing = Double.NaN, localHold = Double.NaN;
    public double angle, speed, target, worldError;

    public Turret(HardwareMap map, CalibrationStore store) {
        this(map, store, new BatteryVoltage(map));
    }

    public Turret(HardwareMap map, CalibrationStore store, BatteryVoltage battery) {
        this.battery = battery;
        motor = map.get(DcMotorEx.class, RobotConfig.TURRET_MOTOR);
        encoder = map.get(DcMotorEx.class, RobotConfig.TURRET_ENCODER);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        geometry = store.read(CalibrationStore.HOME, 0, 0, 0, 0);
        plant = Plant.load(store);
        friction = new Friction(store, plant);
        setCoupling(store.read(CalibrationStore.TURRET_COUPLED_DYNAMICS, 0, 0, 0, 0));
        range = new Range(geometry[2]);
        double[] stored = store.readExact(CalibrationStore.TURRET_PID, 3);
        savedPid = stored != null && stored[0] >= 0 && stored[1] >= 0 && stored[2] >= 0
                && (stored[0] > 0 || stored[1] > 0 || stored[2] > 0);
        setPidGains(savedPid ? stored : plant.pidSeed(plant.baseline()));
        double steadyError = savedPid ? store.read(CalibrationStore.TURRET_PID_STEADY_ERROR, -1)[0] : -1;
        shotAimTolerance = steadyError >= 0
                ? Maths.clip(1.35 * steadyError + Math.toRadians(0.5), Math.toRadians(2), Math.toRadians(6))
                : Math.toRadians(3);
    }

    public void installPlant(Plant model) {
        plant = model;
        friction = new Friction(model);
        tune(model.baseline());
        stop();
    }

    public void tune(double bandwidthScale) {
        setPidGains(plant.pidSeed(bandwidthScale));
    }

    public void setPidGains(double[] gains) {
        this.gains = gains.clone();
        pid = new Maths.Pid(this.gains);
    }

    public double[] pidGains() { return gains.clone(); }
    public boolean usesSavedPid() { return savedPid; }
    public boolean plantIdentified() { return plant.identified(); }
    public boolean hasFrictionMap() { return friction.measured(); }

    public double frictionDuty(double angle, int direction, double speed) {
        return friction.duty(angle, span(), direction, speed);
    }

    public double velocityGain(int direction) { return friction.velocityGain(direction); }
    public double[] coupling() { return coupling.clone(); }

    public void setCoupling(double[] model) {
        coupling = model.clone();
        for (int i = 0; i < 4; i += 2) {
            if (!(coupling[i] > 0 && coupling[i + 1] >= 0)) coupling[i] = coupling[i + 1] = 0;
        }
    }

    public int rawTicks() {
        return encoder.getController().getMotorCurrentPosition(encoder.getPortNumber());
    }

    public boolean calibrated() {
        return Math.abs(geometry[0]) == 1 && Math.abs(geometry[1]) == 1
                && geometry[2] >= Math.PI / 2 && geometry[2] <= Maths.TURN
                && geometry[3] >= 0 && geometry[3] <= geometry[2] + Math.toRadians(10);
    }

    public double span() { return geometry[2]; }
    public int homeTick() { return homeTick; }
    public boolean isHomed() { return homed; }
    public boolean encoderReady() { return homed && encoderFresh; }
    public boolean encoderFault() { return calibrated() && (!homed || encoderAge >= 0.10); }
    public boolean motorResponding() { return motorResponding; }
    public double boreRelative() { return Maths.wrap(angle - geometry[3]); }

    public double forwardAngle() {
        return Maths.clip(geometry[3], RobotConfig.TURRET_MARGIN,
                Math.max(RobotConfig.TURRET_MARGIN, span() - RobotConfig.TURRET_MARGIN));
    }

    public double previewAngle(double bearing) {
        double next = bearing + geometry[3];
        next += Maths.TURN * Math.ceil((RobotConfig.TURRET_MARGIN - next) / Maths.TURN);
        return next <= span() - RobotConfig.TURRET_MARGIN ? next : Double.NaN;
    }

    public double initRelativeAngle() {
        try {
            int ticks = rawTicks();
            if (initTick == null) initTick = ticks;
            return geometry[0] * (ticks - initTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
        } catch (RuntimeException unavailable) {
            return Double.NaN;
        }
    }

    public void homeAtRight() {
        try {
            int right = rawTicks();
            home(right, right);
        } catch (RuntimeException unavailable) {
            homed = encoderFresh = false;
        }
    }

    public void homeAtRight(int right) {
        try {
            home(right, rawTicks());
        } catch (RuntimeException unavailable) {
            homed = encoderFresh = false;
        }
    }

    private void home(int right, int current) {
        homeTick = right;
        lastTick = current;
        homed = true;
        encoderFresh = true;
        encoderAge = 0;
        angle = geometry[0] * (lastTick - homeTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
        speed = 0;
        stop();
    }

    public void sample(double dt) {
        battery.update(Maths.time());
        if (!homed) return;
        double wasStillFor = stillSeconds;
        stillSeconds = 0;
        encoderAge += dt;
        boolean consecutive = encoderFresh;
        encoderFresh = false;
        try {
            int ticks = rawTicks();
            double change = geometry[0] * (ticks - lastTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
            if (Math.abs(change) > Math.toRadians(6) + 40 * Math.min(0.10, encoderAge)) return;
            angle = geometry[0] * (ticks - homeTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
            double measuredSpeed = change / encoderAge;
            speed += encoderAge / (0.026 + encoderAge) * (measuredSpeed - speed);
            if (consecutive && encoderAge <= RobotConfig.SHOT_TURRET_STILL_SECONDS
                    && Math.max(Math.abs(measuredSpeed), Math.abs(speed)) <= RobotConfig.SHOT_TURRET_STILL_RATE) {
                stillSeconds = wasStillFor + encoderAge;
            }
            int direction = Math.abs(speed) > 0.13 ? (speed >= 0 ? 1 : -1) : (appliedPower >= 0 ? 1 : -1);
            double drag = frictionDuty(angle, direction, speed);
            if (Math.abs(speed) < 0.13) drag = Math.min(drag, Math.abs(appliedPower));
            observer.update(angle, speed, encoderAge, appliedPower, direction * drag,
                    plant.kV(direction), plant.timeConstant(direction));
            lastTick = ticks;
            encoderFresh = true;
            encoderAge = 0;
        } catch (RuntimeException unavailable) {
            // The next loop tries again with the same encoder origin.
        }
    }

    /** Actively returns the turret to chassis forward (boreRelative = 0) and holds locked with BRAKE. */
    public void lockAtForward(double dt) {
        if (!calibrated() || !homed) {
            stop();
            return;
        }
        worldBearing = Double.NaN;
        localHold = Double.NaN;
        fieldTracking = false;
        trackedGoal = 0;
        range.reset();
        previousCoupledRate = coupledAcceleration = filteredStick = 0;
        double forwardTarget = forwardAngle();
        target = forwardTarget;
        worldError = Maths.wrap(forwardTarget - angle);
        control(forwardTarget, 0, dt);
    }

    public boolean atForwardZero() {
        return calibrated() && encoderReady() && motorResponding
                && Math.abs(boreRelative()) <= RobotConfig.SHOT_AIM_TOLERANCE;
    }

    public void aim(PinpointPose pose, int goalNumber, double bearing, double lineRate, double yawTrim, double stick, double dt) {
        if (!calibrated() || !homed) {
            stop();
            return;
        }
        fieldTracking = pose.headingUsable();
        if (!fieldTracking) {
            if (!Double.isFinite(localHold)) {
                localHold = Maths.clip(angle, 0, span());
                range.reset();
                pid.reset();
                holding = false;
            }
            previousCoupledRate = coupledAcceleration = filteredStick = 0;
            target = localHold;
            control(localHold, 0, dt);
            return;
        }
        localHold = Double.NaN;
        double actualBearing = pose.heading + boreRelative();
        if (!Double.isFinite(worldBearing)) worldBearing = actualBearing - yawTrim;

        double stabilized = worldBearing + yawTrim - pose.heading + geometry[3];
        if (goalNumber != 0 && trackedGoal != goalNumber) {
            trackedGoal = goalNumber;
            range.reset();
            pid.reset();
        }
        double correction = goalNumber == 0 ? 0 : Maths.wrap(bearing - worldBearing);
        worldBearing = Maths.wrap(worldBearing + correction);
        double goalRate = goalNumber == 0 ? 0 : Maths.clip(lineRate, -5, 5);
        double relativeRate = goalRate - pose.yawRate;
        int toward = relativeRate >= 0 ? 1 : -1;
        double lookahead = Maths.clip(0.23 + plant.delay(toward), 0.24, 0.36);
        boolean wasRewinding = range.rewinding;
        double lead = Maths.clip(goalRate * plant.delay(toward), -Math.toRadians(5), Math.toRadians(5));
        target = range.target(stabilized + correction + lead, angle, relativeRate, lookahead);
        worldError = Maths.wrap(worldBearing + yawTrim - actualBearing);
        if (range.rewinding != wasRewinding) {
            pid.reset();
            holding = false;
        }
        boolean tracking = range.rewinding ? range.rewindTargetMoves() : range.reachable;
        double desiredRate = tracking ? relativeRate : 0;
        double rate = -pose.yawRate;
        double acceleration = Maths.clip((rate - previousCoupledRate) / dt, -35, 35);
        coupledAcceleration += dt / (0.055 + dt) * (acceleration - coupledAcceleration);
        previousCoupledRate = rate;
        filteredStick += dt / (0.11 + dt) * (stick - filteredStick);
        double anticipatedRate = tracking ? 0.55 * (filteredStick - stick) : 0;
        control(target, desiredRate, tracking ? rate : 0, tracking ? coupledAcceleration : 0,
                anticipatedRate, 0, dt);
        if (range.rewinding && !range.predicted && pose.headingValid && encoderReady() && motorResponding
                && Math.abs(target - angle) <= RobotConfig.SHOT_AIM_TOLERANCE
                && Math.abs(speed - desiredRate) < 0.32) {
            range.finishRewind();
        }
    }

    public boolean aimReady() { return aimReady(shotAimTolerance); }
    public double shotAimTolerance() { return shotAimTolerance; }

    public boolean aimReady(double tolerance) {
        return calibrated() && encoderReady() && motorResponding && fieldTracking && range.reachable && !range.rewinding
                && Math.abs(worldError) <= tolerance;
    }

    public boolean stableForShot() {
        return calibrated() && encoderReady() && motorResponding
                && stillSeconds >= RobotConfig.SHOT_TURRET_STILL_SECONDS;
    }

    public boolean rewinding() { return range.rewinding; }
    public boolean rewindPredicted() { return range.rewinding && range.predicted; }
    public boolean targetReachable() { return range.reachable; }
    public double output() { return appliedPower; }
    public double actualDuty() { return actualDuty; }
    public double observerBias() { return observer.bias; }
    public double brakeDuty() { return brakeDuty; }
    public double controlTarget() { return controlTarget; }

    public void control(double targetAngle, double targetRate, double dt) {
        control(targetAngle, targetRate, 0, 0, 0, 0, dt);
    }

    public void controlWithMeasuredBreakaway(double targetAngle, double targetRate, double breakaway, double dt) {
        control(targetAngle, targetRate, 0, 0, 0, breakaway, dt);
    }

    private void control(double targetAngle, double targetRate, double chassisRate, double chassisAcceleration,
                         double anticipatedRate, double measuredBreakaway, double dt) {
        controlTarget = targetAngle;
        double error = targetAngle - observer.angle;
        boolean quiet = Math.abs(targetRate) < 0.075 && Math.abs(anticipatedRate) < 0.10
                && Math.abs(chassisAcceleration) < 0.7;
        boolean within = Math.abs(targetAngle - angle) < Math.toRadians(holding ? 0.85 : 0.38)
                && Math.abs(speed) < (holding ? 0.16 : 0.075);
        holding = quiet && within;
        if (holding) {
            pid.reset();
            brakeDuty = 0;
            power(0);
            return;
        }
        double reference = Maths.clip(targetRate, -RobotConfig.TURRET_MAX_RATE, RobotConfig.TURRET_MAX_RATE);
        double demand = reference + error * 3.3;
        int direction = demand >= 0 ? 1 : -1;
        double referenceKv = velocityGain(reference == 0 ? direction : (reference > 0 ? 1 : -1));
        double ks = frictionDuty(angle, direction, speed);
        double frictionScale = Maths.clip(Math.abs(demand) / 0.11, 0, 1);
        double breakaway = Math.abs(speed) <= 0.12 && measuredBreakaway * direction > 0
                ? Math.abs(measuredBreakaway) : 0;
        double frictionPower = Math.copySign(Math.max(ks * frictionScale, breakaway), demand);
        double rateError = observer.velocity - reference;
        int brakeDirection = rateError >= 0 ? 1 : -1;
        brakeDuty = encoderFresh ? Brake.duty(error, rateError, frictionPower, appliedPower,
                plant.kV(brakeDirection), plant.timeConstant(brakeDirection), plant.delay(brakeDirection)) : 0;
        double feedforward = frictionPower + brakeDuty + referenceKv * reference;
        double disturbanceRate = Math.abs(chassisRate) > 0.10 ? chassisRate : anticipatedRate;
        int side = Friction.side(Math.abs(disturbanceRate) > 0.10
                ? (disturbanceRate > 0 ? 1 : -1) : direction);
        double fittedKv = coupling[side * 2];
        double fittedKa = coupling[side * 2 + 1];
        double velocityCorrection = fittedKv > 0 ? (fittedKv - referenceKv) * chassisRate : 0;
        if (fittedKv <= 0) {
            int chassisDirection = side == 0 ? 1 : -1;
            fittedKa = plant.kV(chassisDirection) * plant.timeConstant(chassisDirection);
        }
        feedforward += Maths.clip(velocityCorrection + (fittedKv > 0 ? fittedKv : referenceKv) * anticipatedRate
                + fittedKa * chassisAcceleration, -0.50, 0.50);
        double low = -powerLimit(-1), high = powerLimit(1);
        double lower = Maths.clip(appliedPower - 8.0 * dt, low, high);
        double upper = Maths.clip(appliedPower + 8.0 * dt, low, high);
        power(pid.update(error, rateError, feedforward, dt, lower, upper,
                Math.abs(brakeDuty) < 0.045));
    }

    public void power(double logicalPower) {
        lastPower = Maths.clip(logicalPower, -powerLimit(-1), powerLimit(1));
        writePower(battery.duty(lastPower, OUTPUT_LIMIT) * geometry[1]);
        if (motorResponding) appliedPower = battery.nominal(actualDuty * geometry[1]);
    }

    public double powerLimit(int direction) {
        if (!homed || !calibrated() || encoderAge >= 0.10) return 0;
        double remaining = direction > 0 ? span() - angle : angle;
        double horizon = Maths.clip(0.5 * plant.timeConstant(direction) + plant.delay(direction), 0.08, 0.30);
        double stoppingAllowance = Math.max(0, direction * speed) * horizon;
        double usable = remaining - RobotConfig.TURRET_MARGIN - stoppingAllowance;
        double available = battery.nominal(OUTPUT_LIMIT);
        double taper = available * Maths.clip(usable / Math.toRadians(12), 0, 1);
        double breakaway = Math.min(available, frictionDuty(angle, direction, 0) + 0.035);
        double limit = Math.max(taper, breakaway * Maths.clip(usable / RobotConfig.SHOT_AIM_TOLERANCE, 0, 1));
        return encoderFresh ? limit : Math.min(limit, 0.15);
    }

    public void teachingMode(boolean free) {
        stop();
        try {
            motor.setZeroPowerBehavior(free ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (RuntimeException unavailable) {
            motorResponding = false;
        }
    }

    public void motorPulse(double power) {
        battery.update(Maths.time());
        writePower(battery.duty(Maths.clip(power, -0.22, 0.22), OUTPUT_LIMIT));
    }

    private void writePower(double power) {
        try {
            motor.setPower(power);
            actualDuty = power;
            motorResponding = true;
        } catch (RuntimeException unavailable) {
            motorResponding = false;
        }
    }

    public void stop() {
        writePower(0);
        if (motorResponding) appliedPower = 0;
        lastPower = 0;
        brakeDuty = 0;
        worldBearing = Double.NaN;
        localHold = Double.NaN;
        fieldTracking = false;
        trackedGoal = 0;
        previousCoupledRate = coupledAcceleration = filteredStick = 0;
        range.reset();
        holding = false;
        stillSeconds = 0;
        pid.reset();
        observer.reset(angle, speed);
    }
}
