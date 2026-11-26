package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.localization.PinpointPose;

/**
 * Ported DECODE turret controller adapted to BIOBUZZ wiring, shared encoder, and cable-limited travel.
 * Uses smooth heading estimation, outer position->velocity PIDF, target acceleration feedforward,
 * 12 V compensation, and predictive unwind braking.
 */
public final class DecodeTurret implements TurretController {
    private static final double OBSERVER_RESET_RATE = 38.0; // rad/s
    private static final double OUT_OF_TRAVEL_MARGIN = Math.toRadians(20.0);
    private static final double SOFT_HOME_MIN_SECONDS = 0.12;
    private static final double SOFT_HOME_STALL_SECONDS = 0.16;
    private static final double SOFT_HOME_TIMEOUT_SECONDS = 4.50;
    private static final double SOFT_HOME_STALL_RATE = 0.12;
    private static final double SOFT_HOME_MIN_DUTY = 0.20;
    private static final double SOFT_HOME_MAX_DUTY = 0.32;

    private final DcMotorEx motor;
    private final DcMotorEx encoder;
    private final CalibrationStore store;
    private final BatteryVoltage battery;
    private final double[] geometry;
    private final DecodePidController pid = new DecodePidController();

    private double[] savedParams;
    private boolean hasSavedParams;

    private int homeTick, lastTick, trackedGoal;
    private Integer initTick;
    private boolean homed, encoderFresh, fieldTracking, motorResponding = true, outOfTravel, softHoming;
    private double softHomeElapsed, softHomeStallSeconds;
    private int softHomeLastTick;
    private double encoderAge, appliedPower, actualDuty, lastPower;
    private double stillSeconds;

    private boolean isTurretFilterInitialized;
    private double filteredTurretRelDeg;
    private double filteredTurretVelDeg;
    private double lastTargetVelDeg;
    private double filteredTargetAccelDeg;
    private boolean hasLastTargetVel;

    private boolean isSmoothHeadingInit;
    private double smoothHeadingDeg;
    private double filteredRobotOmegaDeg;

    private boolean isCurrentlyUnwinding;
    private int unwindDirection;
    private boolean targetReachable;

    private double worldBearingRad = Double.NaN;
    private double localHold = Double.NaN;
    private double controlTarget;
    private double lastBrakeDistDeg;
    private double lastTargetVelCmdDeg;

    public double angle, speed, target, worldError;

    public DecodeTurret(HardwareMap map, CalibrationStore store) {
        this(map, store, new BatteryVoltage(map));
    }

    public DecodeTurret(HardwareMap map, CalibrationStore store, BatteryVoltage battery) {
        this.store = store;
        this.battery = battery;
        motor = map.get(DcMotorEx.class, RobotConfig.TURRET_MOTOR);
        encoder = map.get(DcMotorEx.class, RobotConfig.TURRET_ENCODER);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        geometry = store.read(CalibrationStore.HOME, 0, 0, 0, 0);
        reloadSavedParams();
    }

    public void reloadSavedParams() {
        double[] stored = store.readExact(CalibrationStore.DECODE_TURRET_PARAMS, 6);
        hasSavedParams = validDecodeParams(stored);
        savedParams = hasSavedParams ? stored.clone() : null;
    }

    private static boolean validDecodeParams(double[] values) {
        return values != null && values.length == 6 && Maths.finite(values)
                && values[0] >= 0.0 && values[1] > 0.0 && values[2] >= 0.0
                && values[3] >= 0.0 && values[4] >= 0.0 && values[5] >= 8.0;
    }

    public boolean hasSavedParams() { return hasSavedParams; }

    public String paramsSource() {
        if (DecodeTurretConfig.USE_DASHBOARD_FF_OVERRIDE) return "Dashboard覆盖";
        return hasSavedParams ? "02B实测" : "默认保底";
    }

    public double[] pidGains() {
        return new double[]{
                DecodeTurretConfig.TURRET_kP,
                DecodeTurretConfig.TURRET_kI,
                DecodeTurretConfig.TURRET_kD,
                DecodeTurretConfig.TURRET_kF
        };
    }

    public double[] ffPbParams() {
        return new double[]{
                activeKs(), activeKv(), activeKa(),
                activeKLin(), activeKQuad(), activeTuningVoltage()
        };
    }

    private boolean useSaved() {
        return !DecodeTurretConfig.USE_DASHBOARD_FF_OVERRIDE && hasSavedParams;
    }

    private double activeKs() {
        return Math.max(0.0, useSaved() ? savedParams[0] : DecodeTurretConfig.TURRET_kS);
    }

    private double activeKv() {
        return Math.max(0.0, useSaved() ? savedParams[1] : DecodeTurretConfig.TURRET_kV);
    }

    private double activeKa() {
        return Math.max(0.0, useSaved() ? savedParams[2] : DecodeTurretConfig.TURRET_kA);
    }

    private double activeKLin() {
        return Math.max(0.0, useSaved() ? savedParams[3] : DecodeTurretConfig.TURRET_kLinearBraking);
    }

    private double activeKQuad() {
        return Math.max(0.0, useSaved() ? savedParams[4] : DecodeTurretConfig.TURRET_kQuadraticFriction);
    }

    private double activeTuningVoltage() {
        double v = useSaved() ? savedParams[5] : DecodeTurretConfig.TUNING_VOLTAGE;
        return Math.max(8.0, Double.isFinite(v) ? v : BatteryVoltage.NOMINAL_VOLTS);
    }

    public int rawTicks() {
        return encoder.getController().getMotorCurrentPosition(encoder.getPortNumber());
    }

    @Override public boolean calibrated() {
        return Math.abs(geometry[0]) == 1 && Math.abs(geometry[1]) == 1
                && geometry[2] >= Math.PI / 2 && geometry[2] <= Maths.TURN
                && geometry[3] >= 0 && geometry[3] <= geometry[2] + Math.toRadians(10);
    }

    @Override public double span() { return geometry[2]; }
    @Override public int homeTick() { return homeTick; }
    @Override public int lastTick() { return lastTick; }
    @Override public boolean isHomed() { return homed; }
    @Override public boolean outOfTravel() { return outOfTravel; }
    @Override public boolean softHoming() { return softHoming; }
    @Override public boolean encoderReady() { return homed && encoderFresh; }
    @Override public boolean motorResponding() { return motorResponding; }
    @Override public double boreRelative() { return Maths.wrap(angle - geometry[3]); }

    public double forwardAngle() {
        return Maths.clip(geometry[3], RobotConfig.TURRET_MARGIN,
                Math.max(RobotConfig.TURRET_MARGIN, Math.min(span(), Maths.TURN) - RobotConfig.TURRET_MARGIN));
    }

    @Override public double initRelativeAngle() {
        try {
            int ticks = rawTicks();
            if (initTick == null) initTick = ticks;
            return geometry[0] * (ticks - initTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
        } catch (RuntimeException unavailable) {
            return Double.NaN;
        }
    }

    @Override public boolean isInheritedHomePlausible(int savedHomeTick, boolean hasLastTick, int lastKnownTick) {
        if (!calibrated()) return false;
        try {
            int current = rawTicks();
            double angleRad = geometry[0] * (current - savedHomeTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
            if (!Double.isFinite(angleRad) || angleRad < -Math.toRadians(25)
                    || angleRad > span() + Math.toRadians(25)) {
                RobotLog.ww("BioBuzzTurret",
                        "Inherited home rejected (out of range): current=%d home=%d angle=%.1f deg",
                        current, savedHomeTick, Math.toDegrees(angleRad));
                return false;
            }
            if (current == 0 && Math.abs(savedHomeTick) > 200) {
                RobotLog.ww("BioBuzzTurret",
                        "Inherited home rejected (counter reset to exact zero): current=%d last=%d (has=%b) home=%d",
                        current, lastKnownTick, hasLastTick, savedHomeTick);
                return false;
            }
            return true;
        } catch (RuntimeException unavailable) {
            return false;
        }
    }

    @Override public void homeAtRight() {
        try {
            int right = rawTicks();
            home(right, right);
        } catch (RuntimeException unavailable) {
            homed = encoderFresh = false;
        }
    }

    @Override public void homeAtRight(int right) {
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
        outOfTravel = false;
        softHoming = false;
        softHomeElapsed = softHomeStallSeconds = 0;
        encoderFresh = true;
        encoderAge = 0;
        angle = geometry[0] * (lastTick - homeTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
        speed = 0;
        resetFilters();
        stop();
    }

    @Override public boolean startSoftHome() {
        if (!calibrated()) return false;
        try {
            int ticks = rawTicks();
            stop();
            lastTick = softHomeLastTick = ticks;
            encoderFresh = true;
            encoderAge = 0;
            softHoming = true;
            softHomeElapsed = 0;
            softHomeStallSeconds = 0;
            RobotLog.ii("BioBuzzTurret", "Soft right-stop homing started (ticks=%d)", ticks);
            return true;
        } catch (RuntimeException unavailable) {
            return false;
        }
    }

    @Override public void cancelSoftHome() {
        if (softHoming) {
            softHoming = false;
            stop();
        }
    }

    @Override public Turret.SoftHomeResult updateSoftHome(double dt) {
        if (!softHoming || !calibrated()) {
            softHoming = false;
            return Turret.SoftHomeResult.FAILED;
        }
        softHomeElapsed += dt;
        if (!encoderFresh && encoderAge >= 0.30) {
            RobotLog.ww("BioBuzzTurret", "Soft right-stop homing aborted: encoder unreadable (age=%.0f ms)",
                    1000 * encoderAge);
            softHoming = false;
            stop();
            return Turret.SoftHomeResult.FAILED;
        }
        double duty = Maths.clip(activeKs() + 0.14, SOFT_HOME_MIN_DUTY, SOFT_HOME_MAX_DUTY);
        writePower(battery.duty(-duty, RobotConfig.TURRET_MAX_POWER) * geometry[1]);
        if (motorResponding) appliedPower = battery.nominal(actualDuty * geometry[1]);
        if (!motorResponding || softHomeElapsed >= SOFT_HOME_TIMEOUT_SECONDS) {
            RobotLog.ww("BioBuzzTurret", "Soft right-stop homing timed out or motor fault (%.2f s, motor=%b)",
                    softHomeElapsed, motorResponding);
            softHoming = false;
            stop();
            return Turret.SoftHomeResult.FAILED;
        }
        double tickDeltaRad = Math.abs(geometry[0] * (lastTick - softHomeLastTick) / RobotConfig.TICKS_PER_TURRET_RADIAN);
        softHomeLastTick = lastTick;
        double instantRate = tickDeltaRad / Math.max(dt, 0.001);
        if (softHomeElapsed >= SOFT_HOME_MIN_SECONDS && encoderFresh
                && Math.max(instantRate, Math.abs(speed)) <= SOFT_HOME_STALL_RATE) {
            softHomeStallSeconds += dt;
            if (softHomeStallSeconds >= SOFT_HOME_STALL_SECONDS) {
                softHoming = false;
                home(lastTick, lastTick);
                RobotLog.ii("BioBuzzTurret", "Soft right-stop homing succeeded: homeTick=%d (%.2f s)",
                        homeTick, softHomeElapsed);
                return Turret.SoftHomeResult.SUCCEEDED;
            }
        } else {
            softHomeStallSeconds = 0;
        }
        return Turret.SoftHomeResult.RUNNING;
    }

    @Override public void sample(double dt) {
        battery.update(Maths.time());
        if (!homed && !softHoming) return;
        double wasStillFor = stillSeconds;
        stillSeconds = 0;
        encoderAge += dt;
        boolean consecutive = encoderFresh;
        encoderFresh = false;
        try {
            int ticks = rawTicks();
            double stepDt = Math.max(encoderAge, 0.001);
            double change = geometry[0] * (ticks - lastTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
            double measuredSpeed = change / stepDt;
            lastTick = ticks;
            encoderFresh = true;
            encoderAge = 0;
            if (homed) {
                angle = geometry[0] * (ticks - homeTick) / RobotConfig.TICKS_PER_TURRET_RADIAN;
                outOfTravel = angle < -OUT_OF_TRAVEL_MARGIN || angle > span() + OUT_OF_TRAVEL_MARGIN;
            }
            if (stepDt > 0.25 || Math.abs(measuredSpeed) > OBSERVER_RESET_RATE) {
                if (homed) {
                    RobotLog.ww("BioBuzzTurret",
                            "Large encoder step accepted (DECODE filter reset): ticks=%d home=%d delta=%+.1f deg dt=%.0f ms",
                            ticks, homeTick, Math.toDegrees(change), 1000 * stepDt);
                }
                resetFilters();
                pid.reset();
                speed = 0;
                return;
            }

            double rawRelDeg = Math.toDegrees(angle - geometry[3]);
            double rawVelDeg = Math.toDegrees(measuredSpeed);
            double posAlpha = Maths.clip(DecodeTurretConfig.TURRET_FILTER_ALPHA, 0.05, 1.0);
            double velAlpha = Maths.clip(DecodeTurretConfig.TURRET_VEL_FILTER_ALPHA, 0.05, 1.0);
            if (!isTurretFilterInitialized) {
                filteredTurretRelDeg = rawRelDeg;
                filteredTurretVelDeg = 0.0;
                lastTargetVelDeg = 0.0;
                filteredTargetAccelDeg = 0.0;
                hasLastTargetVel = false;
                isTurretFilterInitialized = true;
            } else {
                filteredTurretRelDeg = posAlpha * rawRelDeg + (1.0 - posAlpha) * filteredTurretRelDeg;
                filteredTurretVelDeg = velAlpha * rawVelDeg + (1.0 - velAlpha) * filteredTurretVelDeg;
            }
            speed = Math.toRadians(filteredTurretVelDeg);

            if (homed && consecutive && stepDt <= RobotConfig.SHOT_TURRET_STILL_SECONDS
                    && Math.max(Math.abs(measuredSpeed), Math.abs(speed)) <= RobotConfig.SHOT_TURRET_STILL_RATE) {
                stillSeconds = wasStillFor + stepDt;
            }
        } catch (RuntimeException unavailable) {
            // Next loop keeps accumulating encoderAge and continues cleanly from lastTick.
        }
    }

    @Override public void lockAtForward(double dt) {
        if (!calibrated() || !homed) {
            stop();
            return;
        }
        worldBearingRad = Double.NaN;
        localHold = Double.NaN;
        fieldTracking = false;
        trackedGoal = 0;
        isSmoothHeadingInit = false;
        isCurrentlyUnwinding = false;
        unwindDirection = 0;
        targetReachable = true;

        double forwardTarget = forwardAngle();
        target = forwardTarget;
        worldError = Maths.wrap(forwardTarget - angle);
        double targetRelDeg = Math.toDegrees(forwardTarget - geometry[3]);
        controlRelDeg(targetRelDeg, 0.0, dt);
    }

    @Override public boolean atForwardZero() {
        return calibrated() && encoderReady() && motorResponding
                && Math.abs(boreRelative()) <= RobotConfig.SHOT_AIM_TOLERANCE;
    }

    @Override public void aim(PinpointPose pose, int goalNumber, double bearing, double lineRate,
                              double yawTrim, double stick, double dt) {
        if (!calibrated() || !homed) {
            stop();
            return;
        }
        fieldTracking = pose.headingUsable();
        if (!fieldTracking) {
            isSmoothHeadingInit = false;
            if (!Double.isFinite(localHold)) {
                double minTravel = RobotConfig.TURRET_MARGIN;
                double maxTravel = Math.max(minTravel, Math.min(span(), Maths.TURN) - RobotConfig.TURRET_MARGIN);
                localHold = Maths.clip(angle, minTravel, maxTravel);
                pid.reset();
                isCurrentlyUnwinding = false;
                unwindDirection = 0;
                hasLastTargetVel = false;
            }
            targetReachable = true;
            target = localHold;
            worldError = 0.0;
            controlRelDeg(Math.toDegrees(localHold - geometry[3]), 0.0, dt);
            return;
        }

        localHold = Double.NaN;
        double currentHeadingDeg = Math.toDegrees(pose.heading);
        double robotOmegaDeg = Math.toDegrees(pose.yawRate);
        double omegaAlpha = Maths.clip(DecodeTurretConfig.OMEGA_FILTER_ALPHA, 0.0, 1.0);
        double headingAlpha = Maths.clip(DecodeTurretConfig.HEADING_CORRECTION_ALPHA, 0.0, 1.0);

        if (!isSmoothHeadingInit) {
            smoothHeadingDeg = currentHeadingDeg;
            filteredRobotOmegaDeg = robotOmegaDeg;
            isSmoothHeadingInit = true;
        } else {
            filteredRobotOmegaDeg += omegaAlpha * (robotOmegaDeg - filteredRobotOmegaDeg);
            smoothHeadingDeg = wrapDeg(smoothHeadingDeg + filteredRobotOmegaDeg * dt);
            double headingError = wrapDeg(currentHeadingDeg - smoothHeadingDeg);
            smoothHeadingDeg = wrapDeg(smoothHeadingDeg + headingAlpha * headingError);
        }

        double actualBearingRad = pose.heading + boreRelative();
        if (!Double.isFinite(worldBearingRad)) worldBearingRad = Maths.wrap(actualBearingRad - yawTrim);
        if (goalNumber != 0 && trackedGoal != goalNumber) {
            trackedGoal = goalNumber;
            pid.reset();
        }

        double translationalOmegaDeg = 0.0;
        if (goalNumber != 0) {
            worldBearingRad = Maths.wrap(bearing);
            translationalOmegaDeg = Math.toDegrees(Maths.clip(lineRate, -5.0, 5.0));
        }

        worldError = Maths.wrap(worldBearingRad + yawTrim - actualBearingRad);
        double compensatedTargetAbsDeg = Math.toDegrees(worldBearingRad + yawTrim)
                + translationalOmegaDeg * DecodeTurretConfig.TURRET_LATENCY;

        double minRelDeg = Math.toDegrees(RobotConfig.TURRET_MARGIN - geometry[3]);
        double maxRelDeg = Math.toDegrees(Math.min(span(), Maths.TURN) - RobotConfig.TURRET_MARGIN - geometry[3]);
        if (maxRelDeg < minRelDeg) {
            double mid = 0.5 * (minRelDeg + maxRelDeg);
            minRelDeg = maxRelDeg = mid;
        }

        double desiredRelWrapped = wrapDeg(compensatedTargetAbsDeg - smoothHeadingDeg);
        double[] candidates = new double[5];
        int candCount = 0;
        for (int k = -2; k <= 2; k++) {
            double c = desiredRelWrapped + 360.0 * k;
            if (c >= minRelDeg && c <= maxRelDeg) {
                candidates[candCount++] = c;
            }
        }

        if (!isTurretFilterInitialized) {
            filteredTurretRelDeg = Math.toDegrees(angle - geometry[3]);
            filteredTurretVelDeg = Math.toDegrees(speed);
            isTurretFilterInitialized = true;
        }

        double targetRelDeg;
        if (candCount >= 2) {
            targetReachable = true;
            if (isCurrentlyUnwinding && unwindDirection != 0) {
                targetRelDeg = candidates[0];
                double bestScore = unwindDirection > 0 ? Double.NEGATIVE_INFINITY : Double.POSITIVE_INFINITY;
                for (int i = 0; i < candCount; i++) {
                    if (unwindDirection > 0 && candidates[i] > bestScore) {
                        bestScore = candidates[i];
                        targetRelDeg = candidates[i];
                    } else if (unwindDirection < 0 && candidates[i] < bestScore) {
                        bestScore = candidates[i];
                        targetRelDeg = candidates[i];
                    }
                }
            } else {
                targetRelDeg = candidates[0];
                double bestDist = Math.abs(candidates[0] - filteredTurretRelDeg);
                for (int i = 1; i < candCount; i++) {
                    double d = Math.abs(candidates[i] - filteredTurretRelDeg);
                    if (d < bestDist) {
                        bestDist = d;
                        targetRelDeg = candidates[i];
                    }
                }
            }
        } else if (candCount == 1) {
            targetReachable = true;
            targetRelDeg = candidates[0];
            if (!isCurrentlyUnwinding && Math.abs(targetRelDeg - filteredTurretRelDeg) > 180.0) {
                isCurrentlyUnwinding = true;
                unwindDirection = targetRelDeg >= filteredTurretRelDeg ? 1 : -1;
                pid.reset();
            }
        } else {
            targetReachable = false;
            if (isCurrentlyUnwinding && unwindDirection != 0) {
                targetRelDeg = unwindDirection > 0 ? maxRelDeg : minRelDeg;
            } else {
                double nearCurrent = filteredTurretRelDeg + wrapDeg(desiredRelWrapped - filteredTurretRelDeg);
                targetRelDeg = Maths.clip(nearCurrent, minRelDeg, maxRelDeg);
            }
        }

        if (isCurrentlyUnwinding
                && Math.abs(targetRelDeg - filteredTurretRelDeg) < DecodeTurretConfig.UNWIND_EXIT_ANGLE_DEG
                && Math.abs(filteredTurretVelDeg) < DecodeTurretConfig.UNWIND_EXIT_VEL_DEG_S) {
            isCurrentlyUnwinding = false;
            unwindDirection = 0;
        }

        target = Math.toRadians(targetRelDeg) + geometry[3];
        double feedforwardVelDeg = (targetReachable || isCurrentlyUnwinding)
                ? (-filteredRobotOmegaDeg + translationalOmegaDeg) : 0.0;
        controlRelDeg(targetRelDeg, feedforwardVelDeg, dt);
    }

    /** Direct closed-loop control to a travel angle [0, span] in radians (used by 02B verification). */
    public void controlAngleRad(double targetTravelRad, double dt) {
        if (!calibrated() || !homed) {
            stop();
            return;
        }
        double minTravel = RobotConfig.TURRET_MARGIN;
        double maxTravel = Math.max(minTravel, Math.min(span(), Maths.TURN) - RobotConfig.TURRET_MARGIN);
        double clampedRad = Maths.clip(targetTravelRad, minTravel, maxTravel);
        target = clampedRad;
        worldError = Maths.wrap(clampedRad - angle);
        double targetRelDeg = Math.toDegrees(clampedRad - geometry[3]);
        if (!isTurretFilterInitialized) {
            filteredTurretRelDeg = Math.toDegrees(angle - geometry[3]);
            filteredTurretVelDeg = Math.toDegrees(speed);
            isTurretFilterInitialized = true;
        }
        if (!isCurrentlyUnwinding && Math.abs(targetRelDeg - filteredTurretRelDeg) > 180.0) {
            isCurrentlyUnwinding = true;
            unwindDirection = targetRelDeg >= filteredTurretRelDeg ? 1 : -1;
            pid.reset();
        } else if (isCurrentlyUnwinding
                && Math.abs(targetRelDeg - filteredTurretRelDeg) < DecodeTurretConfig.UNWIND_EXIT_ANGLE_DEG
                && Math.abs(filteredTurretVelDeg) < DecodeTurretConfig.UNWIND_EXIT_VEL_DEG_S) {
            isCurrentlyUnwinding = false;
            unwindDirection = 0;
        }
        controlRelDeg(targetRelDeg, 0.0, dt);
    }

    private void controlRelDeg(double targetRelDeg, double feedforwardVelDeg, double dt) {
        if (!isTurretFilterInitialized) {
            filteredTurretRelDeg = Math.toDegrees(angle - geometry[3]);
            filteredTurretVelDeg = Math.toDegrees(speed);
            isTurretFilterInitialized = true;
        }
        controlTarget = Math.toRadians(targetRelDeg) + geometry[3];

        double kS = activeKs();
        double kV = activeKv();
        double kA = activeKa();
        double kLin = activeKLin();
        double kQuad = activeKQuad();
        double tuningV = activeTuningVoltage();

        double brakingDistDeg = 0.0;
        double predictedRelDeg = filteredTurretRelDeg;
        if (isCurrentlyUnwinding) {
            double absVel = Math.abs(filteredTurretVelDeg);
            brakingDistDeg = (kLin * absVel) + (kQuad * absVel * absVel);
            predictedRelDeg = filteredTurretRelDeg + Math.signum(filteredTurretVelDeg) * brakingDistDeg;
        }
        lastBrakeDistDeg = brakingDistDeg;

        double pidOutputVelDeg;
        if (!isCurrentlyUnwinding
                && Math.abs(targetRelDeg - filteredTurretRelDeg) <= DecodeTurretConfig.ANGLE_DEADBAND_DEG
                && Math.abs(feedforwardVelDeg) < 0.5) {
            pid.reset();
            pidOutputVelDeg = 0.0;
        } else {
            pid.setPIDF(DecodeTurretConfig.TURRET_kP, DecodeTurretConfig.TURRET_kI,
                    DecodeTurretConfig.TURRET_kD, DecodeTurretConfig.TURRET_kF);
            pidOutputVelDeg = pid.calculate(predictedRelDeg, targetRelDeg, dt);
        }

        double finalTargetVelDeg = pidOutputVelDeg + feedforwardVelDeg;
        lastTargetVelCmdDeg = finalTargetVelDeg;

        double rawTargetAccelDeg = 0.0;
        if (hasLastTargetVel && dt > 0.0001) {
            rawTargetAccelDeg = (finalTargetVelDeg - lastTargetVelDeg) / dt;
        }
        lastTargetVelDeg = finalTargetVelDeg;
        hasLastTargetVel = true;

        double cmdAccelAlpha = Maths.clip(DecodeTurretConfig.TURRET_CMD_ACCEL_FILTER_ALPHA, 0.01, 1.0);
        filteredTargetAccelDeg = cmdAccelAlpha * rawTargetAccelDeg + (1.0 - cmdAccelAlpha) * filteredTargetAccelDeg;

        double ksSign = Math.abs(finalTargetVelDeg) > DecodeTurretConfig.KS_VEL_DEADBAND_DEG_S
                ? Math.signum(finalTargetVelDeg) : 0.0;
        double nominalPower = (filteredTargetAccelDeg * kA)
                + (finalTargetVelDeg * kV)
                + (ksSign * kS);

        if (!homed || !calibrated() || encoderAge >= 0.10) {
            nominalPower = 0.0;
        } else {
            if (!encoderFresh) {
                nominalPower = Maths.clip(nominalPower, -0.15, 0.15);
            }
            if (DecodeTurretConfig.ENABLE_SOFT_LIMIT_GUARD) {
                double minAngleRad = RobotConfig.TURRET_MARGIN;
                double maxAngleRad = Math.min(span(), Maths.TURN) - RobotConfig.TURRET_MARGIN;
                double absVelDeg = Math.abs(filteredTurretVelDeg);
                double predStopRad = Math.toRadians((kLin * absVelDeg) + (kQuad * absVelDeg * absVelDeg));
                if (nominalPower < 0.0) {
                    if (angle <= minAngleRad || (filteredTurretVelDeg < 0.0 && angle - minAngleRad <= predStopRad)) {
                        nominalPower = 0.0;
                    }
                } else if (nominalPower > 0.0) {
                    if (angle >= maxAngleRad || (filteredTurretVelDeg > 0.0 && maxAngleRad - angle <= predStopRad)) {
                        nominalPower = 0.0;
                    }
                }
            }
        }

        double maxPower = Maths.clip(DecodeTurretConfig.TURRET_MAX_POWER, 0.0, RobotConfig.TURRET_MAX_POWER);
        double liveVolts = Math.max(8.0, battery.volts());
        double duty = Maths.clip(nominalPower * (tuningV / liveVolts), -maxPower, maxPower);
        lastPower = nominalPower;
        writePower(duty * geometry[1]);
        if (motorResponding) appliedPower = battery.nominal(actualDuty * geometry[1]);
    }

    @Override public boolean aimReady() { return aimReady(shotAimTolerance()); }

    @Override public double shotAimTolerance() {
        return Math.toRadians(Maths.clip(DecodeTurretConfig.SHOT_AIM_TOLERANCE_DEG, 0.5, 15.0));
    }

    @Override public boolean aimReady(double tolerance) {
        return calibrated() && encoderReady() && motorResponding && fieldTracking
                && targetReachable && !isCurrentlyUnwinding && Math.abs(worldError) <= tolerance;
    }

    @Override public boolean stableForShot() {
        return calibrated() && encoderReady() && motorResponding
                && stillSeconds >= RobotConfig.SHOT_TURRET_STILL_SECONDS;
    }

    @Override public boolean rewinding() { return isCurrentlyUnwinding; }
    @Override public boolean rewindPredicted() { return false; }
    @Override public boolean targetReachable() { return targetReachable; }
    @Override public double angle() { return angle; }
    @Override public double speed() { return speed; }
    @Override public double target() { return target; }
    @Override public double worldError() { return worldError; }
    @Override public double output() { return appliedPower; }
    @Override public double actualDuty() { return actualDuty; }

    public double targetVelDegPerSec() { return lastTargetVelCmdDeg; }
    public double targetAccelDegPerSecSq() { return filteredTargetAccelDeg; }
    public double brakeDistDeg() { return lastBrakeDistDeg; }
    public double lastNominalPower() { return lastPower; }
    public double controlTarget() { return controlTarget; }

    @Override public void teachingMode(boolean free) {
        stop();
        try {
            motor.setZeroPowerBehavior(free ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (RuntimeException unavailable) {
            motorResponding = false;
        }
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

    private void resetFilters() {
        isTurretFilterInitialized = false;
        filteredTurretRelDeg = Math.toDegrees(angle - geometry[3]);
        filteredTurretVelDeg = 0.0;
        lastTargetVelDeg = 0.0;
        filteredTargetAccelDeg = 0.0;
        hasLastTargetVel = false;
        lastBrakeDistDeg = 0.0;
        lastTargetVelCmdDeg = 0.0;
    }

    @Override public void stop() {
        writePower(0);
        if (motorResponding) appliedPower = 0;
        lastPower = 0;
        worldBearingRad = Double.NaN;
        localHold = Double.NaN;
        fieldTracking = false;
        trackedGoal = 0;
        isSmoothHeadingInit = false;
        isCurrentlyUnwinding = false;
        unwindDirection = 0;
        stillSeconds = 0;
        pid.reset();
        resetFilters();
    }

    private static double wrapDeg(double deg) {
        double wrapped = deg % 360.0;
        if (wrapped > 180.0) return wrapped - 360.0;
        if (wrapped < -180.0) return wrapped + 360.0;
        return wrapped;
    }

    /** Lightweight self-contained PIDF matching FTCLib PIDFController behavior without FTCLib dependency. */
    private static final class DecodePidController {
        private static final double INTEGRAL_VEL_LIMIT_DEG_S = 180.0;
        private double kP, kI, kD, kF;
        private double integralSum;
        private double prevError;
        private boolean initialized;

        void setPIDF(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        double calculate(double pv, double sp, double dt) {
            double error = sp - pv;
            double safeDt = Math.max(dt, 1e-4);
            if (!initialized) {
                prevError = error;
                initialized = true;
            }
            if (Math.abs(kI) > 1e-9) {
                double maxIntegral = INTEGRAL_VEL_LIMIT_DEG_S / Math.abs(kI);
                integralSum = Maths.clip(integralSum + error * safeDt, -maxIntegral, maxIntegral);
            } else {
                integralSum = 0.0;
            }
            double derivative = (error - prevError) / safeDt;
            prevError = error;
            return kP * error + kI * integralSum + kD * derivative + kF * sp;
        }

        void reset() {
            integralSum = 0.0;
            prevError = 0.0;
            initialized = false;
        }
    }
}
