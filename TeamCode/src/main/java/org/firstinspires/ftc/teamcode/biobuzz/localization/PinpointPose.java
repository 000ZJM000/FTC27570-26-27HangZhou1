package org.firstinspires.ftc.teamcode.biobuzz.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.Register;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.BatteryVoltage;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;

/**
 * Field-frame wrapper around goBILDA Pinpoint with Control Hub internal IMU heading backup.
 * Pinpoint remains the sole primary authority whenever healthy; Control Hub IMU supplies heading
 * and yaw rate during Pinpoint bad reads, I2C backoffs, or rejected jumps.
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
    private static final double HEADING_SLACK = Math.toRadians(4.5);
    private static final double MAX_POSITION_RATE = 5000.0;
    private static final double POSITION_SLACK_MM = 120.0;
    private static final double HUB_SYNC_PERIOD = 0.05;
    private static final double HUB_RETRY_SECONDS = 0.50;
    private static final double HUB_VERIFY_WINDOW_SECONDS = 1.0;
    private static final double HUB_VERIFY_MIN_TURN = Math.toRadians(10.0);
    private static final double HUB_VERIFY_ABS_TOL = Math.toRadians(4.0);
    private static final double HUB_VERIFY_REL_TOL = 0.30;
    private static final int HUB_MISMATCH_LIMIT = 2;
    private static final String POSITION_REANCHOR = "Pinpoint 位置需要重新对齐，回起点按 Y";

    private final GoBildaPinpointDriver device;
    private final IMU hubImu;
    private final BatteryVoltage battery;
    private boolean started, configured, anchored, positionAnchored, needsAnchor, positionNeedsAnchor, podsValid;
    private boolean waitingReset, calibratingSeen;
    private double resetStartedAt, setPoseGraceUntil = Double.NEGATIVE_INFINITY;
    private double sensorYaw, localX, localY;
    private double previousTime = Double.NEGATIVE_INFINITY, previousX, previousY;
    private double lastHeadingTime = Double.NEGATIVE_INFINITY, lastPinpointHeadingTime = Double.NEGATIVE_INFINITY;
    private double lastHeadingSampleTime;
    private double lastYawRate, observedYaw = Double.NaN, observedYawRate = Double.NaN;
    private boolean headingRejected, positionRejected;
    private double nextRead = Double.NEGATIVE_INFINITY, readRetryDelay = READ_RETRY_SECONDS;
    private double nextScopeRestore = Double.NEGATIVE_INFINITY;
    private int rawX, rawY;

    private LogoFacingDirection configuredLogoDir;
    private UsbFacingDirection configuredUsbDir;
    private boolean hubImuAvailable, hubSynced, hubImuVerified, hubImuMismatch;
    private double hubYawOffset, lastHubYaw = Double.NaN;
    private double nextHubSync = Double.NEGATIVE_INFINITY, nextHubRetry = Double.NEGATIVE_INFINITY;
    private double verifyPinpointYaw, verifyHubYaw, verifyTime = Double.NEGATIVE_INFINITY;
    private int hubMismatchStreak, hubReadFailures;
    private String hubImuError = "";

    public boolean headingValid, positionValid, usingHubImu;
    public double forward, left, heading, yawRate;
    public double forwardVelocity, leftVelocity;
    public String status = "等待 Pinpoint";
    public String lastFault = "";
    public int faults;
    public int badReads, recoveredReads, badReadStreak;
    public int hubFallbackEvents, hubFallbackFrames;
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
        hubImu = map.tryGet(IMU.class, RobotConfig.HUB_IMU);
        this.battery = battery;
        configureHubImu(RobotConfig.HUB_IMU_LOGO_DIR, RobotConfig.HUB_IMU_USB_DIR);
    }

    /**
     * Attaches to an already-running Pinpoint (e.g. after Autonomous) and aligns driver configuration
     * and 10-register bulk scope WITHOUT resetting position or recalibrating the IMU.
     */
    public boolean attachExisting() {
        if (device == null) return false;
        started = headingValid = podsValid = positionValid = usingHubImu = false;
        configured = false;
        waitingReset = calibratingSeen = false;
        anchored = positionAnchored = needsAnchor = positionNeedsAnchor = false;
        setPoseGraceUntil = Double.NEGATIVE_INFINITY;
        lastHeadingTime = lastPinpointHeadingTime = previousTime = Double.NEGATIVE_INFINITY;
        lastHeadingSampleTime = 0;
        lastYawRate = yawRate = 0;
        forwardVelocity = leftVelocity = 0;
        headingRejected = positionRejected = false;
        hubSynced = false;
        nextHubSync = Double.NEGATIVE_INFINITY;
        resetHubVerify();
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
        started = headingValid = podsValid = positionValid = usingHubImu = false;
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
            lastHeadingTime = lastPinpointHeadingTime = previousTime = Double.NEGATIVE_INFINITY;
            lastHeadingSampleTime = 0;
            lastYawRate = yawRate = 0;
            forwardVelocity = leftVelocity = 0;
            headingRejected = positionRejected = false;
            hubSynced = false;
            nextHubSync = Double.NEGATIVE_INFINITY;
            resetHubVerify();
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
        boolean wasUsingHub = usingHubImu;
        headingValid = podsValid = positionValid = false;
        usingHubImu = false;
        observedYaw = observedYawRate = Double.NaN;
        yawRate = 0;
        double readStart = Maths.time();
        battery.update(readStart);
        ensureHubImuOrientation();
        if (device == null) {
            if (!tryHubFallback(readStart, "找不到 " + RobotConfig.PINPOINT, wasUsingHub)) {
                status = "找不到 " + RobotConfig.PINPOINT;
            }
            return;
        }
        if (!started) return;
        if (readStart < nextRead) {
            tryHubFallback(readStart, lastFault.isEmpty() ? "Pinpoint 退避等待" : lastFault, wasUsingHub);
            return;
        }

        try {
            if (!configured) {
                configure();
            } else if (firmwareVersion == 0) {
                int version = device.getDeviceVersion();
                if (version < 1) {
                    failRead("等待 Pinpoint 固件读数", readStart);
                    tryHubFallback(readStart, "Pinpoint 固件坏读", wasUsingHub);
                    return;
                }
                firmwareVersion = version;
            }
            // A Pinpoint restart resets custom bulk layout. Always allow restoration in both Auto and TeleOp.
            if (firmwareVersion >= 3 && badReadStreak >= 4 && readStart >= nextScopeRestore) {
                nextScopeRestore = readStart + BULK_SCOPE_RETRY_SECONDS;
                restoreBulkScope();
                tryHubFallback(readStart, "Pinpoint 重建布局", wasUsingHub);
                return;
            }
            device.update();
            DeviceStatus state = device.getDeviceStatus();
            double now = Maths.time();
            if (state == DeviceStatus.FAULT_BAD_READ) {
                failRead(state.name(), now);
                tryHubFallback(now, state.name(), wasUsingHub);
                return;
            }
            boolean recoveredFromBadRead = badReadStreak > 0;
            if (recoveredFromBadRead) recoveredReads++;
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
                lastHeadingTime = lastPinpointHeadingTime = previousTime = Double.NEGATIVE_INFINITY;
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
                tryHubFallback(now, state.name(), wasUsingHub);
                return;
            }
            if (ready) {
                rawX = device.getEncoderX();
                rawY = device.getEncoderY();
            }
            double sampleTime = readStart + 0.5 * (now - readStart);
            updateHeading(sampleTime, now, recoveredFromBadRead);
            if (!headingValid) {
                tryHubFallback(now, "Pinpoint 航向跳变", wasUsingHub);
                return;
            }
            if (needsAnchor) {
                status = "Pinpoint 需要重新对齐，回起点按 Y";
                tryHubFallback(now, "Pinpoint 需重对齐", wasUsingHub);
            } else {
                if (wasUsingHub) {
                    RobotLog.ii("BioBuzzPinpoint", "Pinpoint recovered -> primary (heading=%+.1f deg)",
                            Math.toDegrees(heading));
                    resetHubVerify();
                }
                status = state.name();
                syncHubImu(sampleTime, now, wasUsingHub || !hubSynced);
            }
            if (ready) updatePosition(sampleTime, now);
            else reportFault(state.name());
        } catch (RuntimeException unavailable) {
            headingValid = podsValid = positionValid = false;
            double now = Maths.time();
            String reason = "Pinpoint 读取失败 " + unavailable.getClass().getSimpleName();
            failRead(reason, now);
            tryHubFallback(now, reason, wasUsingHub);
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

    private void updateHeading(double sampleTime, double now, boolean skipConsistencyCheck) {
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
        boolean inconsistent = !inGrace && !skipConsistencyCheck && !needsAnchor
                && Double.isFinite(lastPinpointHeadingTime)
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
            if (now - lastPinpointHeadingTime >= HEADING_GRACE && !anchored) {
                lastPinpointHeadingTime = Double.NEGATIVE_INFINITY;
                lastHeadingTime = Double.NEGATIVE_INFINITY;
            }
            reportFault("Pinpoint 航向跳变");
            return;
        }
        if (headingRejected) recoveredHeadingJumps++;
        headingRejected = false;
        sensorYaw = nextYaw;
        lastHeadingSampleTime = sampleTime;
        lastPinpointHeadingTime = now;
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
            lastHeadingTime = lastPinpointHeadingTime = previousTime = now;
            lastHeadingSampleTime = now;
            setPoseGraceUntil = now + SET_POSE_GRACE_SECONDS;
            anchored = true;
            needsAnchor = positionNeedsAnchor = false;
            headingRejected = positionRejected = false;
            usingHubImu = false;
            headingValid = podsValid = positionAnchored = positionValid = true;
            status = DeviceStatus.READY.name();
            resetHubVerify();
            syncHubImu(now, now, true);
            return true;
        } catch (RuntimeException failure) {
            reportFault("Pinpoint 设置坐标失败");
            return false;
        }
    }

    /** Adopts Pinpoint's live hardware field coordinates without touching any position register. */
    public boolean acceptCurrentHardwarePose() {
        if (!canAnchorPosition() || waitingReset) return false;
        double now = Maths.time();
        forward = localX;
        left = localY;
        heading = Maths.wrap(sensorYaw);
        anchored = positionAnchored = true;
        needsAnchor = positionNeedsAnchor = false;
        headingRejected = positionRejected = false;
        usingHubImu = false;
        lastHeadingTime = lastPinpointHeadingTime = now;
        positionValid = true;
        copySample(lastHeadingSampleTime);
        status = DeviceStatus.READY.name();
        resetHubVerify();
        syncHubImu(lastHeadingSampleTime, now, true);
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
    public boolean headingUsable() {
        return (isAnchored() || (usingHubImu && anchored)) && Maths.time() - lastHeadingTime <= HEADING_GRACE;
    }
    public boolean recentlyLocated(double maxAge) {
        return hasPositionAnchor() && Maths.time() - previousTime <= maxAge;
    }

    public boolean hubImuAvailable() { return hubImuAvailable; }
    public boolean hubImuReady() {
        return RobotConfig.ENABLE_HUB_IMU_BACKUP && hubImuAvailable && hubSynced && !hubImuMismatch;
    }
    public boolean hubImuMismatch() { return hubImuMismatch; }
    public boolean hubImuVerified() { return hubImuVerified; }
    public double hubHeading() {
        return hubSynced && Double.isFinite(lastHubYaw) ? Maths.wrap(lastHubYaw + hubYawOffset) : Double.NaN;
    }
    public String imuSource() { return usingHubImu ? "HUB" : "PINPOINT"; }

    public String hubImuSummary() {
        if (!RobotConfig.ENABLE_HUB_IMU_BACKUP) return "已关闭 (Dashboard)";
        if (hubImu == null) return "未配置 " + RobotConfig.HUB_IMU;
        if (!hubImuAvailable) {
            return String.format("不可用 (%s) [%s/%s]",
                    hubImuError.isEmpty() ? "未就绪" : hubImuError, configuredLogoDir, configuredUsbDir);
        }
        if (hubImuMismatch) {
            return String.format("朝向不一致，已禁用补位 [%s/%s]", configuredLogoDir, configuredUsbDir);
        }
        if (usingHubImu) {
            return String.format("正在补位 (%+.1f° / 累计 %d 次)", Math.toDegrees(heading), hubFallbackEvents);
        }
        if (!hubSynced) {
            return String.format("等待 Pinpoint 同步 [%s/%s]", configuredLogoDir, configuredUsbDir);
        }
        return String.format("就绪%s [%s/%s] 补位 %d 次",
                hubImuVerified ? "·已校验" : "", configuredLogoDir, configuredUsbDir, hubFallbackEvents);
    }

    private void ensureHubImuOrientation() {
        if (hubImu == null) return;
        LogoFacingDirection logo = RobotConfig.HUB_IMU_LOGO_DIR != null
                ? RobotConfig.HUB_IMU_LOGO_DIR : LogoFacingDirection.RIGHT;
        UsbFacingDirection usb = RobotConfig.HUB_IMU_USB_DIR != null
                ? RobotConfig.HUB_IMU_USB_DIR : UsbFacingDirection.UP;
        if (logo != configuredLogoDir || usb != configuredUsbDir) {
            configureHubImu(logo, usb);
        }
    }

    private void configureHubImu(LogoFacingDirection logo, UsbFacingDirection usb) {
        LogoFacingDirection resolvedLogo = logo != null ? logo : LogoFacingDirection.RIGHT;
        UsbFacingDirection resolvedUsb = usb != null ? usb : UsbFacingDirection.UP;
        configuredLogoDir = resolvedLogo;
        configuredUsbDir = resolvedUsb;
        hubSynced = false;
        hubImuVerified = false;
        hubImuMismatch = false;
        hubMismatchStreak = 0;
        lastHubYaw = Double.NaN;
        nextHubSync = Double.NEGATIVE_INFINITY;
        nextHubRetry = Double.NEGATIVE_INFINITY;
        resetHubVerify();
        if (hubImu == null) {
            hubImuAvailable = false;
            hubImuError = "未配置 " + RobotConfig.HUB_IMU;
            return;
        }
        try {
            boolean ok = hubImu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(resolvedLogo, resolvedUsb)));
            hubImuAvailable = ok;
            hubImuError = ok ? "" : "初始化返回 false";
        } catch (RuntimeException failure) {
            hubImuAvailable = false;
            hubImuError = failure instanceof IllegalArgumentException
                    ? "朝向配置冲突"
                    : failure.getClass().getSimpleName();
            RobotLog.ww("BioBuzzPinpoint", "Hub IMU init failed (%s/%s): %s",
                    resolvedLogo, resolvedUsb, failure);
        }
    }

    private void resetHubVerify() {
        verifyTime = Double.NEGATIVE_INFINITY;
        verifyPinpointYaw = verifyHubYaw = 0;
    }

    private void syncHubImu(double sampleTime, double now, boolean force) {
        if (!RobotConfig.ENABLE_HUB_IMU_BACKUP || !hubImuAvailable || now < nextHubRetry) return;
        if (!force && now < nextHubSync) return;
        try {
            double hubStart = Maths.time();
            YawPitchRollAngles angles = hubImu.getRobotYawPitchRollAngles();
            double hubEnd = Maths.time();
            double hubYaw = angles.getYaw(AngleUnit.RADIANS);
            if (!Double.isFinite(hubYaw)) return;
            double lag = Maths.clip((hubStart + 0.5 * (hubEnd - hubStart)) - sampleTime, 0.0, 0.02);
            double alignedHeading = Maths.wrap(heading + yawRate * lag);

            if (!force && now >= setPoseGraceUntil) {
                if (!Double.isFinite(verifyTime) || now - verifyTime > HUB_VERIFY_WINDOW_SECONDS) {
                    verifyPinpointYaw = alignedHeading;
                    verifyHubYaw = hubYaw;
                    verifyTime = now;
                } else {
                    double dPin = Maths.wrap(alignedHeading - verifyPinpointYaw);
                    if (Math.abs(dPin) >= HUB_VERIFY_MIN_TURN) {
                        double dHub = Maths.wrap(hubYaw - verifyHubYaw);
                        double err = Math.abs(Maths.wrap(dHub - dPin));
                        double tol = Math.max(HUB_VERIFY_ABS_TOL, HUB_VERIFY_REL_TOL * Math.abs(dPin));
                        if (err <= tol) {
                            hubMismatchStreak = 0;
                            hubImuMismatch = false;
                            hubImuVerified = true;
                        } else {
                            hubMismatchStreak++;
                            if (hubMismatchStreak >= HUB_MISMATCH_LIMIT && !hubImuMismatch) {
                                hubImuMismatch = true;
                                RobotLog.ww("BioBuzzPinpoint",
                                        "Hub IMU orientation mismatch! dPin=%+.1f deg dHub=%+.1f deg (logo=%s usb=%s)",
                                        Math.toDegrees(dPin), Math.toDegrees(dHub),
                                        configuredLogoDir, configuredUsbDir);
                            }
                        }
                        verifyPinpointYaw = alignedHeading;
                        verifyHubYaw = hubYaw;
                        verifyTime = now;
                    }
                }
            }

            hubYawOffset = Maths.wrap(alignedHeading - hubYaw);
            lastHubYaw = hubYaw;
            hubSynced = true;
            hubImuError = "";
            nextHubSync = now + HUB_SYNC_PERIOD;
        } catch (RuntimeException failure) {
            nextHubRetry = now + HUB_RETRY_SECONDS;
            hubImuError = failure.getClass().getSimpleName();
            if (hubReadFailures++ == 0) {
                RobotLog.ww("BioBuzzPinpoint", "Hub IMU sync failed: %s", failure);
            }
        }
    }

    private boolean tryHubFallback(double now, String reason, boolean wasUsingHub) {
        if (!RobotConfig.ENABLE_HUB_IMU_BACKUP || waitingReset || !anchored
                || !hubImuAvailable || !hubSynced || hubImuMismatch || now < nextHubRetry) {
            return false;
        }
        try {
            YawPitchRollAngles angles = hubImu.getRobotYawPitchRollAngles();
            AngularVelocity velocity = hubImu.getRobotAngularVelocity(AngleUnit.RADIANS);
            double hubYaw = angles.getYaw(AngleUnit.RADIANS);
            if (!Double.isFinite(hubYaw)) return false;
            double hubRate = velocity != null ? velocity.zRotationRate : 0;
            if (!Double.isFinite(hubRate) || Math.abs(hubRate) > MAX_YAW_RATE) hubRate = 0;

            lastHubYaw = hubYaw;
            heading = Maths.wrap(hubYaw + hubYawOffset);
            yawRate = hubRate;
            headingValid = true;
            lastHeadingTime = now;
            usingHubImu = true;
            hubImuError = "";
            hubFallbackFrames++;
            if (!wasUsingHub) {
                hubFallbackEvents++;
                RobotLog.ww("BioBuzzPinpoint",
                        "Hub IMU fallback active (%s): hubYaw=%+.1f deg heading=%+.1f deg rate=%+.1f deg/s",
                        reason, Math.toDegrees(hubYaw), Math.toDegrees(heading), Math.toDegrees(hubRate));
            }
            status = "Hub IMU 补位 (" + reason + ")";
            return true;
        } catch (RuntimeException failure) {
            nextHubRetry = now + HUB_RETRY_SECONDS;
            hubImuError = failure.getClass().getSimpleName();
            if (hubReadFailures++ == 0) {
                RobotLog.ww("BioBuzzPinpoint", "Hub IMU fallback read failed: %s", failure);
            }
            return false;
        }
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
