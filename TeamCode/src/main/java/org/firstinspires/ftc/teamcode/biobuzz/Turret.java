package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Turret {
    private static final double OUTPUT_LIMIT = RobotConfig.TURRET_MAX_POWER;
    private final DcMotorEx motor, encoder;
    private final double[] geometry;
    private TurretFriction friction;
    private TurretPlant plant;
    private double[] coupling;
    private final TurretRange range;
    private final TurretRewind rewind = new TurretRewind();
    private final TurretObserver observer = new TurretObserver();
    private Pid pid;
    private double appliedPower, brakeDuty;
    private boolean holding;
    private int homeTick, lastTick, previousGoal;
    private boolean homed, encoderFresh, fieldTracking, motorResponding = true;
    private double encoderAge, goalRate, previousGoalBearing, lastPower;
    private double previousCoupledRate, coupledAcceleration, filteredStick;
    private double worldBearing = Double.NaN, localHold = Double.NaN;
    public double angle, speed, target, worldError;

    public Turret(HardwareMap map, CalibrationStore store) {
        motor = map.get(DcMotorEx.class, RobotConfig.TURRET_MOTOR);
        encoder = map.get(DcMotorEx.class, RobotConfig.TURRET_ENCODER);
        // Standardize the reading without changing m-2's direction or resetting its encoder.
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        geometry = store.read(CalibrationStore.HOME, 0, 0, 0, 0);
        plant = TurretPlant.load(store);
        friction = new TurretFriction(store);
        coupling = store.read(CalibrationStore.TURRET_COUPLED_DYNAMICS, 0, 0, 0, 0);
        range = new TurretRange(geometry[2]);
        setPidGains(store.read(CalibrationStore.TURRET_PID, plant.pidSeed(plant.baseline())));
    }

    public void installPlant(TurretPlant model) {
        plant = model;
        friction = new TurretFriction(model);
        tune(model.baseline());
        stop();
    }

    public void tune(double bandwidthScale) {
        setPidGains(plant.pidSeed(bandwidthScale));
    }

    public void setPidGains(double[] gains) { pid = new Pid(gains); }
    public double frictionDuty(double angle, int direction, double speed) {
        return friction.duty(angle, span(), direction, speed);
    }
    public double velocityGain(int direction) { return friction.velocityGain(direction); }
    public double[] coupling() { return coupling.clone(); }
    public void setCoupling(double[] model) { coupling = model.clone(); }
    public int rawTicks() {
        return encoder.getController().getMotorCurrentPosition(encoder.getPortNumber());
    }
    public boolean calibrated() {
        return Math.abs(geometry[0]) == 1 && Math.abs(geometry[1]) == 1
                && geometry[2] >= Math.PI / 2 && geometry[2] <= Maths.TURN
                && geometry[3] >= 0 && geometry[3] <= geometry[2] + Math.toRadians(10);
    }
    public double span() { return geometry[2]; }

    public boolean isHomed() { return homed; }
    public boolean encoderReady() { return homed && encoderFresh; }
    public boolean motorResponding() { return motorResponding; }
    public double boreRelative() { return Maths.wrap(angle - geometry[3]); }

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
        if (!homed) return;
        encoderAge += dt;
        encoderFresh = false;
        try {
            int ticks = rawTicks();
            double change = geometry[0] * (ticks - lastTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
            if (Math.abs(change) > Math.toRadians(6) + 40 * Math.min(0.10, encoderAge)) return;
            angle = geometry[0] * (ticks - homeTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
            speed += encoderAge / (0.026 + encoderAge) * (change / encoderAge - speed);
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

    public void aim(PinpointPose pose, FieldTargets goal, double stick, double dt) {
        if (!calibrated() || !homed) {
            stop();
            return;
        }
        fieldTracking = pose.headingUsable();
        if (!fieldTracking) {
            rewind.reset();
            if (!Double.isFinite(localHold)) localHold = Maths.clip(angle, 0, span());
            previousGoal = 0;
            goalRate = 0;
            previousCoupledRate = coupledAcceleration = filteredStick = 0;
            control(localHold, 0, dt);
            return;
        }
        localHold = Double.NaN;
        double actualBearing = pose.heading + boreRelative();
        if (!Double.isFinite(worldBearing)) worldBearing = actualBearing;

        // Stage 1: keep the previous world bearing while the drivetrain turns.
        double stabilized = worldBearing - pose.heading + geometry[3];
        // Stage 2: update the desired bearing from the measured position.
        // At the exact pivot/target coincidence, retain the last defined bearing.
        double correction = goal == null || goal.distanceMm < 0.1 ? 0 : Maths.wrap(goal.bearing - worldBearing);
        worldBearing = Maths.wrap(worldBearing + correction);
        updateGoalRate(goal, dt);
        double relativeRate = goalRate - pose.yawRate;
        int toward = relativeRate >= 0 ? 1 : -1;
        double lookahead = Maths.clip(0.23 + plant.delay(toward), 0.24, 0.36);
        boolean wasRewinding = range.rewinding;
        target = range.target(stabilized + correction, angle, relativeRate, lookahead);
        worldError = Maths.wrap(worldBearing - actualBearing);
        if (range.rewinding) {
            if (!wasRewinding) {
                pid.reset();
                holding = false;
            }
            previousCoupledRate = coupledAcceleration = filteredStick = 0;
            double destinationRate = range.rewindTargetMoves() ? relativeRate : 0;
            rewind.update(angle, speed, target, destinationRate, dt);
            control(Maths.clip(rewind.angle, RobotConfig.TURRET_MARGIN, span() - RobotConfig.TURRET_MARGIN),
                    rewind.rate, dt);
            if (encoderReady() && rewind.arrived(angle, speed, target, destinationRate)) {
                range.finishRewind();
                rewind.reset();
            }
            return;
        }
        boolean tracking = range.reachable;
        double desiredRate = tracking ? goalRate - pose.yawRate : 0;
        double rate = tracking ? -pose.yawRate : 0;
        double acceleration = Maths.clip((rate - previousCoupledRate) / dt, -35, 35);
        coupledAcceleration += dt / (0.055 + dt) * (acceleration - coupledAcceleration);
        previousCoupledRate = rate;
        // The joystick transient is only a motor-duty hint. The position and velocity setpoints
        // above come from the measured pose even when joystick and measured rotation disagree.
        filteredStick += dt / (0.11 + dt) * (stick - filteredStick);
        double anticipatedRate = tracking ? 0.55 * (filteredStick - stick) : 0;
        control(target, desiredRate, tracking ? rate : 0, tracking ? coupledAcceleration : 0,
                anticipatedRate, dt);
    }

    private void updateGoalRate(FieldTargets goal, double dt) {
        int goalNumber = goal == null || goal.distanceMm < 0.1 ? 0 : goal.goal;
        if (goalNumber != 0 && previousGoal == goalNumber) {
            double rate = Maths.clip(Maths.wrap(goal.bearing - previousGoalBearing) / dt, -5, 5);
            goalRate += dt / (0.06 + dt) * (rate - goalRate);
        } else goalRate = 0;
        previousGoal = goalNumber;
        if (goal != null) previousGoalBearing = goal.bearing;
    }

    public boolean aimReady() {
        return calibrated() && encoderReady() && motorResponding && fieldTracking && range.reachable && !range.rewinding
                && Math.abs(worldError) <= RobotConfig.SHOT_AIM_TOLERANCE;
    }
    public boolean rewinding() { return range.rewinding; }
    public boolean rewindPredicted() { return range.rewinding && range.predicted; }
    public double output() { return lastPower; }
    public double brakeDuty() { return brakeDuty; }

    public void control(double targetAngle, double targetRate, double dt) {
        control(targetAngle, targetRate, 0, 0, 0, dt);
    }

    private void control(double targetAngle, double targetRate, double chassisRate, double chassisAcceleration,
                         double anticipatedRate, double dt) {
        double error = targetAngle - observer.angle;
        boolean quiet = Math.abs(targetRate) < 0.075 && Math.abs(anticipatedRate) < 0.10
                && Math.abs(chassisAcceleration) < 0.7;
        // Hysteresis avoids repeated breakaway kicks and brake reversals at an essentially fixed target.
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
        double demand = Math.abs(reference) > 0.12 ? reference : error * 3.3;
        int direction = demand >= 0 ? 1 : -1;
        double kV = velocityGain(direction);
        double ks = frictionDuty(angle, direction, speed);
        double frictionScale = Maths.clip(Math.abs(demand) / 0.11, 0, 1);
        double frictionPower = Math.copySign(ks * frictionScale, demand);
        double rateError = observer.velocity - reference;
        int brakeDirection = rateError >= 0 ? 1 : -1;
        brakeDuty = encoderFresh ? TurretBrake.duty(error, rateError, frictionPower, appliedPower,
                plant.kV(brakeDirection), plant.timeConstant(brakeDirection), plant.delay(brakeDirection)) : 0;
        double feedforward = frictionPower + brakeDuty + kV * reference;
        double disturbanceRate = chassisRate + anticipatedRate;
        int side = TurretFriction.side(Math.abs(disturbanceRate) > 0.10
                ? (disturbanceRate > 0 ? 1 : -1) : direction);
        double fittedKv = coupling[side * 2];
        double fittedKa = coupling[side * 2 + 1];
        double velocityCorrection = fittedKv > 0
                ? (fittedKv - velocityGain(side == 0 ? 1 : -1)) * chassisRate : 0;
        if (fittedKv <= 0) {
            int chassisDirection = side == 0 ? 1 : -1;
            fittedKa = plant.kV(chassisDirection) * plant.timeConstant(chassisDirection);
        }
        feedforward += Maths.clip(velocityCorrection + (fittedKv > 0 ? fittedKv : kV) * anticipatedRate
                + fittedKa * chassisAcceleration, -0.50, 0.50);
        double disturbance = Maths.clip(Math.abs(speed) / 0.45, 0, 1) * observer.bias;
        // Feed the real limits into the only PID so its integrator also respects the duty slew.
        double low = -powerLimit(-1), high = powerLimit(1);
        double lower = Maths.clip(appliedPower - 8.0 * dt, low, high);
        double upper = Maths.clip(appliedPower + 8.0 * dt, low, high);
        power(pid.update(error, rateError, feedforward - disturbance, dt, lower, upper,
                Math.abs(brakeDuty) < 0.045));
    }

    // Positive power always moves left after the geometry test. Refuse outward commands near an end.
    public void power(double logicalPower) {
        lastPower = Maths.clip(logicalPower, -powerLimit(-1), powerLimit(1));
        writePower(lastPower * geometry[1]);
        if (motorResponding) appliedPower = lastPower;
    }

    private double powerLimit(int direction) {
        if (!homed || !calibrated() || encoderAge >= 0.10) return 0;
        double remaining = direction > 0 ? span() - angle : angle;
        double horizon = Maths.clip(0.5 * plant.timeConstant(direction) + plant.delay(direction), 0.08, 0.30);
        double stoppingAllowance = Math.max(0, direction * speed) * horizon;
        double usable = remaining - RobotConfig.TURRET_MARGIN - stoppingAllowance;
        double limit = OUTPUT_LIMIT * Maths.clip(usable / Math.toRadians(12), 0, 1);
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
        writePower(Maths.clip(power, -0.22, 0.22));
    }

    private void writePower(double power) {
        try {
            motor.setPower(power);
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
        previousGoal = 0;
        goalRate = 0;
        previousCoupledRate = coupledAcceleration = filteredStick = 0;
        range.reset();
        rewind.reset();
        holding = false;
        pid.reset();
        observer.reset(angle, speed);
    }
}
