package org.firstinspires.ftc.teamcode.biobuzz.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.Register;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;

/**
 * Field-frame wrapper around goBILDA Pinpoint.
 * Uses Pinpoint's native world coordinates (setPosition) and native 1500 Hz velocity registers,
 * supports non-resetting attachExisting() across Auto -> TeleOp, and guards against async reset races.
 */
public final class PinpointPose {
    private static final double HEADING_GRACE = 0.50;
    private static final double POSITION_GRACE = 0.25;
    private static final double SET_POSE_GRACE_SECONDS = 0.15;
    private static final double RESET_MIN_WAIT_SECONDS = 0.35;
    private static final double READ_RETRY_SECONDS = 0.05;
    private static final double MAX_READ_RETRY_SECONDS = 0.50;
    private static final double BULK_SCOPE_RETRY_SECONDS = 2.0;
    private static final double MAX_YAW_RATE = 18.0;
    private static final double HEADING_SLACK = Math.toRadians(2.0);
    private static final double MAX_POSITION_RATE = 5000.0;
    private static final double POSITION_SLACK_MM = 60.0;
    private static final String POSITION_REANCHOR = "Pinpoint 位置需要重新对齐，回起点按 Y";

    private final GoBildaPinpointDriver device;
    private final BatteryVoltage battery;
    private boolean started, configured, anchored, positionAnchored, needsAnchor, positionNeedsAnchor, podsValid;
    private boolean waitingReset, calibratingSeen;
    private double resetStartedAt, setPoseGraceUntil = Double.NEGATIVE_INFINITY;
    private double sensorYaw, localX, localY;
    private double previousTime = Double.NEGATIVE_INFINITY, previousX, previousY;
    private double lastHeadingTime = Double.NEGATIVE_INFINITY, lastHeadingSampleTime;
    private double lastYawRate, observedYaw = Double.NaN, observedYawRate = Double.NaN;
    private boolean headingRejected, positionRejected;
    private double nextRead = Double.NEGATIVE_INFINITY, readRetryDelay = READ_RETRY_SECONDS;
    private double nextScopeRestore = Double.NEGATIVE_INFINITY;
    private int rawX, rawY;

    public boolean headingValid, positionValid;
    public double forward, left, heading, yawRate;
    public double forwardVelocity, leftVelocity;
    public String status = "等待 Pinpoint";
    public String lastFault = "";
    public int faults;
    public int badReads, recoveredReads, badReadStreak;
    public int headingJumps, recoveredHeadingJumps;
    public double lastJumpAngle, lastJumpSeconds, lastJumpRate;
    public int positionJumps;
    public double lastPositionJumpMm, lastPositionJumpSeconds;
    public int firmwareVersion;

    public PinpointPose(HardwareMap map) {
        this(map, new BatteryVoltage(map));
    }

    public PinpointPose(HardwareMap map, BatteryVoltage battery) {
        device = map.tryGet(GoBildaPinpointDriver.class, RobotConfig.PINPOINT);
        this.battery = battery;
    }

    /**
     * Attaches to an already-running Pinpoint (e.g. after Autonomous) and aligns driver configuration
     * and 10-register bulk scope WITHOUT resetting position or recalibrating the IMU.
     */
    public boolean attachExisting() {
        if (device == null) return false;
        started = headingValid = podsValid = positionValid = false;
        configured = false;
        waitingReset = calibratingSeen = false;
        anchored = positionAnchored = needsAnchor = positionNeedsAnchor = false;
        setPoseGraceUntil = Double.NEGATIVE_INFINITY;
        lastHeadingTime = previousTime = Double.NEGATIVE_INFINITY;
        lastHeadingSampleTime = 0;
        lastYawRate = yawRate = 0;
        forwardVelocity = leftVelocity = 0;
        headingRejected = positionRejected = false;
        clearReadBackoff();
        try {
            configure();
            started = true;
            return true;
        } catch (RuntimeException failure) {
            started = true;
            status = "Pinpoint 恢复连接中";
            return false;
        }
    }

    /** Configures Pinpoint and triggers async resetPosAndIMU() for Autonomous / Calibration start. */
    public boolean startImu() {
        if (device == null) return false;
        started = headingValid = podsValid = positionValid = false;
        configured = false;
        try {
            configure();
            device.resetPosAndIMU();
            started = true;
            waitingReset = true;
            calibratingSeen = false;
            resetStartedAt = Maths.time();
            anchored = positionAnchored = needsAnchor = positionNeedsAnchor = false;
            setPoseGraceUntil = Double.NEGATIVE_INFINITY;
            lastHeadingTime = previousTime = Double.NEGATIVE_INFINITY;
            lastHeadingSampleTime = 0;
            lastYawRate = yawRate = 0;
            forwardVelocity = leftVelocity = 0;
            headingRejected = positionRejected = false;
            clearReadBackoff();
            status = "等待 Pinpoint IMU 校零";
            return true;
        } catch (RuntimeException failure) {
            status = "Pinpoint 初始化失败";
            return false;
        }
    }

    private void configure() {
        firmwareVersion = RobotConfig.configurePinpoint(device);
        if (firmwareVersion >= 3) restoreBulkScope();
        configured = true;
    }

    private void restoreBulkScope() {
        device.setBulkReadScope(
                Register.DEVICE_STATUS, Register.LOOP_TIME,
                Register.X_ENCODER_VALUE, Register.Y_ENCODER_VALUE,
                Register.X_POSITION, Register.Y_POSITION, Register.H_ORIENTATION,
                Register.X_VELOCITY, Register.Y_VELOCITY, Register.H_VELOCITY);
    }

    public void update() {
        headingValid = podsValid = positionValid = false;
        observedYaw = observedYawRate = Double.NaN;
        yawRate = 0;
        if (device == null) {
            status = "找不到 " + RobotConfig.PINPOINT;
            return;
        }
        if (!started) return;
        double readStart = Maths.time();
        battery.update(readStart);
        if (readStart < nextRead) return;

        try {
            if (!configured) {
                configure();
            } else if (firmwareVersion == 0) {
                int version = device.getDeviceVersion();
                if (version < 1) {
                    failRead("等待 Pinpoint 固件读数", readStart);
                    return;
                }
                firmwareVersion = version;
            }
            // A Pinpoint restart resets custom bulk layout. Always allow restoration in both Auto and TeleOp.
            if (firmwareVersion >= 3 && badReadStreak >= 4 && readStart >= nextScopeRestore) {
                nextScopeRestore = readStart + BULK_SCOPE_RETRY_SECONDS;
                restoreBulkScope();
                return;
            }
            device.update();
            DeviceStatus state = device.getDeviceStatus();
            double now = Maths.time();
            if (state == DeviceStatus.FAULT_BAD_READ) {
                failRead(state.name(), now);
                return;
            }
            if (badReadStreak > 0) recoveredReads++;
            clearReadBackoff();

            if (state == DeviceStatus.CALIBRATING || state == DeviceStatus.NOT_READY) {
                if (waitingReset) {
                    calibratingSeen = true;
                    status = "Pinpoint IMU 校零中";
                    return;
                }
                if (anchored) needsAnchor = true;
            }
            if (waitingReset) {
                if (!calibratingSeen && now - resetStartedAt < RESET_MIN_WAIT_SECONDS) {
                    status = "等待 Pinpoint 进入校零";
                    return;
                }
                waitingReset = false;
                lastHeadingTime = previousTime = Double.NEGATIVE_INFINITY;
            }
            if (state == DeviceStatus.FAULT_IMU_RUNAWAY && anchored) {
                needsAnchor = true;
            }
            boolean ready = state == DeviceStatus.READY;
            if (!ready && state != DeviceStatus.FAULT_X_POD_NOT_DETECTED
                    && state != DeviceStatus.FAULT_Y_POD_NOT_DETECTED
                    && state != DeviceStatus.FAULT_NO_PODS_DETECTED) {
                status = state.name();
                if (anchored || state == DeviceStatus.FAULT_IMU_RUNAWAY) reportFault(status);
                return;
            }
            if (ready) {
                rawX = device.getEncoderX();
                rawY = device.getEncoderY();
            }
            double sampleTime = readStart + 0.5 * (now - readStart);
            updateHeading(sampleTime, now);
            if (!headingValid) return;
            status = needsAnchor ? "Pinpoint 需要重新对齐，回起点按 Y" : state.name();
            if (ready) updatePosition(sampleTime, now);
            else reportFault(state.name());
        } catch (RuntimeException unavailable) {
            headingValid = podsValid = positionValid = false;
            failRead("Pinpoint 读取失败 " + unavailable.getClass().getSimpleName(), Maths.time());
        }
    }

    private void failRead(String reason, double now) {
        if (badReadStreak == 0) {
            RobotLog.ww("BioBuzzPinpoint", "%s V_bat=%.2f V measured=%b", reason, battery.volts(), battery.measured());
        }
        badReads++;
        badReadStreak++;
        nextRead = now + readRetryDelay;
        readRetryDelay = Math.min(MAX_READ_RETRY_SECONDS, 2 * readRetryDelay);
        reportFault(reason);
    }

    private void clearReadBackoff() {
        badReadStreak = 0;
        nextRead = nextScopeRestore = Double.NEGATIVE_INFINITY;
        readRetryDelay = READ_RETRY_SECONDS;
    }

    private void updateHeading(double sampleTime, double now) {
        double nextYaw = device.getHeading(UnnormalizedAngleUnit.RADIANS);
        double rate = device.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        observedYaw = Maths.wrap(nextYaw);
        observedYawRate = rate;
        double elapsed = sampleTime - lastHeadingSampleTime;
        double delta = Maths.wrap(nextYaw - sensorYaw);
        boolean ratesValid = Maths.finite(nextYaw, rate) && Math.abs(rate) <= MAX_YAW_RATE;
        double expected = 0.5 * (lastYawRate + rate) * elapsed;
        double slack = HEADING_SLACK + 0.35 * Math.abs(rate - lastYawRate) * elapsed + 0.15 * Math.abs(expected);
        boolean inGrace = now < setPoseGraceUntil;
        boolean inconsistent = !inGrace && !needsAnchor && Double.isFinite(lastHeadingTime)
                && elapsed > 0 && elapsed < 0.20 && Math.abs(delta - expected) > slack;
        if (!ratesValid || inconsistent) {
            headingJumps++;
            lastJumpAngle = delta;
            lastJumpSeconds = elapsed;
            lastJumpRate = rate;
            if (!headingRejected) {
                RobotLog.ww("BioBuzzPinpoint",
                        "Heading rejected: raw=%+.2f last=%+.2f delta=%+.2f gyro=%+.2f dt=%.0f ms V_bat=%.2f V measured=%b",
                        Math.toDegrees(observedYaw), Math.toDegrees(Maths.wrap(sensorYaw)), Math.toDegrees(delta),
                        Math.toDegrees(rate), 1000 * elapsed, battery.volts(), battery.measured());
            }
            headingRejected = true;
            if (now - lastHeadingTime >= HEADING_GRACE && !anchored) {
                lastHeadingTime = Double.NEGATIVE_INFINITY;
            }
            reportFault("Pinpoint 航向跳变");
            return;
        }
        if (headingRejected) recoveredHeadingJumps++;
        headingRejected = false;
        sensorYaw = nextYaw;
        lastHeadingSampleTime = sampleTime;
        lastHeadingTime = now;
        lastYawRate = rate;
        headingValid = true;
        if (!needsAnchor) {
            heading = observedYaw;
            yawRate = rate;
        }
    }

    private void updatePosition(double sampleTime, double now) {
        localX = device.getPosX(DistanceUnit.MM);
        localY = device.getPosY(DistanceUnit.MM);
        double vx = device.getVelX(DistanceUnit.MM);
        double vy = device.getVelY(DistanceUnit.MM);
        podsValid = Maths.finite(localX, localY);
        if (!hasPositionAnchor()) {
            if (positionNeedsAnchor && !needsAnchor) status = POSITION_REANCHOR;
            else if (!podsValid) reportFault("Pinpoint 位置非有限值");
            return;
        }
        double elapsed = sampleTime - previousTime;
        boolean inGrace = now < setPoseGraceUntil || !Double.isFinite(previousTime);
        double window = Math.min(Math.max(0, elapsed), 0.25);
        double distance = Math.hypot(localX - previousX, localY - previousY);
        boolean jumped = !inGrace && podsValid && elapsed > 0 && elapsed < 0.20
                && distance > MAX_POSITION_RATE * window + POSITION_SLACK_MM;
        if (!podsValid || jumped) {
            if (jumped) {
                positionJumps++;
                lastPositionJumpMm = distance;
                lastPositionJumpSeconds = elapsed;
                if (!positionRejected) {
                    RobotLog.ww("BioBuzzPinpoint", "Position rejected: delta=%.0f mm dt=%.0f ms V_bat=%.2f V measured=%b",
                            distance, 1000 * elapsed, battery.volts(), battery.measured());
                }
            }
            positionRejected = true;
            status = jumped ? "Pinpoint 位置跳变" : "Pinpoint 位置非有限值";
            if (!podsValid) reportFault(status);
            return;
        }
        forward = localX;
        left = localY;
        if (Maths.finite(vx, vy)) {
            forwardVelocity = vx;
            leftVelocity = vy;
        } else {
            forwardVelocity = leftVelocity = 0;
        }
        positionRejected = false;
        copySample(sampleTime);
        positionValid = true;
    }

    /** Writes absolute field coordinates directly into Pinpoint hardware registers. */
    public boolean anchor(double forwardMm, double rightMm, double fieldHeading) {
        if (device == null || !started || waitingReset) return false;
        double now = Maths.time();
        try {
            if (!configured || needsAnchor) {
                configure();
            }
            double wrapped = Maths.wrap(fieldHeading);
            device.setPosition(new Pose2D(DistanceUnit.MM, forwardMm, -rightMm, AngleUnit.RADIANS, wrapped));
            forward = localX = forwardMm;
            left = localY = -rightMm;
            heading = sensorYaw = wrapped;
            forwardVelocity = leftVelocity = yawRate = lastYawRate = 0;
            lastHeadingTime = previousTime = now;
            lastHeadingSampleTime = now;
            setPoseGraceUntil = now + SET_POSE_GRACE_SECONDS;
            anchored = true;
            needsAnchor = positionNeedsAnchor = false;
            headingRejected = positionRejected = false;
            headingValid = podsValid = positionAnchored = positionValid = true;
            status = DeviceStatus.READY.name();
            return true;
        } catch (RuntimeException failure) {
            reportFault("Pinpoint 设置坐标失败");
            return false;
        }
    }

    /** Adopts Pinpoint's live hardware field coordinates without touching any position register. */
    public boolean acceptCurrentHardwarePose() {
        if (!canAnchorPosition() || waitingReset) return false;
        forward = localX;
        left = localY;
        heading = Maths.wrap(sensorYaw);
        anchored = positionAnchored = true;
        needsAnchor = positionNeedsAnchor = false;
        headingRejected = positionRejected = false;
        positionValid = true;
        copySample(lastHeadingSampleTime);
        status = DeviceStatus.READY.name();
        return true;
    }

    /** True if live Pinpoint registers appear freshly reset around (0, 0, 0). */
    public boolean looksHardwareReset() {
        return canAnchorPosition()
                && Math.hypot(localX, localY) < 15.0
                && Math.abs(Maths.wrap(sensorYaw)) < Math.toRadians(1.0);
    }

    public boolean anchorManualCorner() {
        return canAnchorPosition()
                && anchor(RobotConfig.CENTER_FROM_REAR_EDGE_MM, RobotConfig.CENTER_FROM_LEFT_EDGE_MM, 0);
    }

    public boolean isReadyForAnchor() { return started && !waitingReset && canAnchorPosition(); }
    public boolean isAnchored() { return anchored && !needsAnchor; }
    public boolean canAnchorPosition() { return headingValid && podsValid; }
    public boolean hasPositionAnchor() { return positionAnchored && !requiresReanchor(); }
    public boolean requiresReanchor() { return needsAnchor || positionNeedsAnchor; }
    public int encoderX() { return rawX; }
    public int encoderY() { return rawY; }
    public double sensorYaw() { return Maths.wrap(sensorYaw); }
    public double observedYaw() { return observedYaw; }
    public double observedYawRate() { return observedYawRate; }
    public boolean headingUsable() { return isAnchored() && Maths.time() - lastHeadingTime <= HEADING_GRACE; }
    public boolean recentlyLocated(double maxAge) {
        return hasPositionAnchor() && Maths.time() - previousTime <= maxAge;
    }

    private void reportFault(String reason) {
        status = lastFault = reason;
        faults++;
    }

    private void copySample(double now) {
        previousX = localX;
        previousY = localY;
        previousTime = now;
    }
}
