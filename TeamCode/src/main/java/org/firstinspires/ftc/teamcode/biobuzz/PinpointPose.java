package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.Register;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/** The field pose is a rigid transform of the Pinpoint's measured pose. */
public final class PinpointPose {
    private static final double HEADING_GRACE = 0.12;
    private static final double MAX_YAW_RATE = 18;
    private final GoBildaPinpointDriver device;
    private boolean started, anchored, positionAnchored, needsAnchor, podsValid;
    private double sensorYaw, headingShift, localX, localY, originX, originY, fieldForward, fieldLeft;
    private double previousTime, previousX, previousY, lastHeadingTime = Double.NEGATIVE_INFINITY;
    private double lastHeadingReadStart = Double.NEGATIVE_INFINITY;
    private double lastYawRate, rejectedHeadingWindow = Double.POSITIVE_INFINITY;
    private double badReadSince = Double.NaN, nextConfigure;
    private double resetSince = Double.NaN;
    private int rawX, rawY;
    public boolean headingValid, positionValid;
    public double forward, left, heading, yawRate;
    public String status = "等待 Pinpoint";
    public String lastFault = "";
    public int faults;
    public int badReads, recoveredReads, badReadStreak;
    public int headingJumps, recoveredHeadingJumps;
    public double lastJumpAngle, lastJumpSeconds, lastJumpRate;
    public int firmwareVersion;

    public PinpointPose(HardwareMap map) {
        device = map.tryGet(GoBildaPinpointDriver.class, RobotConfig.PINPOINT);
    }

    public boolean startImu() {
        if (device == null) return false;
        started = headingValid = podsValid = positionValid = false;
        try {
            configure();
            device.resetPosAndIMU();
            started = true;
            anchored = positionAnchored = needsAnchor = false;
            headingShift = 0;
            lastHeadingTime = lastHeadingReadStart = Double.NEGATIVE_INFINITY;
            lastYawRate = yawRate = 0;
            rejectedHeadingWindow = Double.POSITIVE_INFINITY;
            resetSince = badReadSince = Double.NaN;
            nextConfigure = 0;
            badReadStreak = 0;
            return true;
        } catch (RuntimeException failure) {
            status = "Pinpoint 初始化失败";
            return false;
        }
    }

    private void configure() {
        int version = RobotConfig.configurePinpoint(device);
        if (version >= 3) {
            device.setBulkReadScope(Register.DEVICE_STATUS, Register.X_ENCODER_VALUE,
                    Register.Y_ENCODER_VALUE, Register.X_POSITION, Register.Y_POSITION, Register.H_ORIENTATION,
                    Register.H_VELOCITY);
        }
        firmwareVersion = version;
    }

    public void update() {
        headingValid = podsValid = positionValid = false;
        if (device == null) {
            status = "找不到 " + RobotConfig.PINPOINT;
            return;
        }
        if (!started) return;
        boolean badRead = false, headingJump = false;
        String failure = null;
        for (int attempt = 0; attempt < 2; attempt++) {
            double readStart = Maths.time();
            try {
                device.update();
                DeviceStatus state = device.getDeviceStatus();
                if (state == DeviceStatus.FAULT_BAD_READ) {
                    if (!badRead) badReads++;
                    badRead = true;
                    failure = state.name();
                    continue;
                }
                double now = Maths.time();
                if (state == DeviceStatus.CALIBRATING || state == DeviceStatus.NOT_READY) {
                    if (!Double.isFinite(resetSince)) resetSince = now;
                    if (anchored && now - resetSince >= HEADING_GRACE) needsAnchor = true;
                } else resetSince = Double.NaN;
                boolean ready = state == DeviceStatus.READY;
                if (!ready && state != DeviceStatus.FAULT_X_POD_NOT_DETECTED
                        && state != DeviceStatus.FAULT_Y_POD_NOT_DETECTED
                        && state != DeviceStatus.FAULT_NO_PODS_DETECTED) {
                    badReadSince = Double.NaN;
                    badReadStreak = 0;
                    status = state.name();
                    if (anchored || state == DeviceStatus.FAULT_IMU_RUNAWAY) reportFault(status);
                    publishHeading(now);
                    return;
                }
                // Compare snapshots within their I²C time windows and across the ±180° seam.
                double observed = device.getHeading(UnnormalizedAngleUnit.RADIANS);
                double rate = device.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
                now = Maths.time();
                if (!Double.isFinite(observed)) {
                    failure = "Pinpoint 航向非有限值";
                    continue;
                }
                double nextYaw = Maths.wrap(observed);
                double elapsed = now - lastHeadingReadStart;
                double delta = Maths.wrap(nextYaw - sensorYaw);
                double nextRate = Double.isFinite(rate) && Math.abs(rate) <= 20 ? rate : 0;
                double allowedRate = Math.min(MAX_YAW_RATE, Math.max(Math.abs(lastYawRate), Math.abs(nextRate)) + 5);
                double window = Math.min(HEADING_GRACE, Math.min(elapsed, rejectedHeadingWindow));
                boolean haveHeading = Double.isFinite(lastHeadingTime);
                if (!needsAnchor && haveHeading && Math.abs(delta) > 0.13 + allowedRate * window) {
                    if (!headingJump) {
                        headingJumps++;
                        lastJumpAngle = delta;
                        lastJumpSeconds = elapsed;
                        lastJumpRate = rate;
                    }
                    // A sample rejected once must not become plausible just by waiting.
                    rejectedHeadingWindow = window;
                    failure = "Pinpoint 航向跳变";
                    headingJump = true;
                    continue;
                }
                if (headingJump) recoveredHeadingJumps++;
                if (badRead) recoveredReads++;
                badReadStreak = 0;
                badReadSince = Double.NaN;
                rejectedHeadingWindow = Double.POSITIVE_INFINITY;
                sensorYaw = nextYaw;
                lastHeadingReadStart = readStart;
                lastHeadingTime = now;
                lastYawRate = nextRate;
                headingValid = true;
                publishHeading(now);
                status = needsAnchor ? "Pinpoint 需要重新对齐，回起点按 Y" : state.name();
                if (ready) updatePosition(now);
                else reportFault(state.name());
                return;
            } catch (RuntimeException unavailable) {
                failure = "Pinpoint 读取失败 " + unavailable.getClass().getSimpleName();
            }
        }
        double now = Maths.time();
        if (headingJump && now - lastHeadingTime >= HEADING_GRACE) {
            needsAnchor = anchored;
            if (!anchored) lastHeadingTime = Double.NEGATIVE_INFINITY;
        }
        if (badRead) {
            badReadStreak++;
            if (!Double.isFinite(badReadSince)) badReadSince = now;
            if (firmwareVersion >= 3 && now - badReadSince >= 0.25 && now >= nextConfigure) {
                nextConfigure = now + 1;
                needsAnchor = anchored;
                try {
                    configure();
                } catch (RuntimeException unavailable) {
                    failure = "Pinpoint 配置恢复失败";
                }
            }
        } else {
            badReadStreak = 0;
            badReadSince = Double.NaN;
        }
        if (failure != null) reportFault(failure);
        publishHeading(now);
    }

    private void updatePosition(double now) {
        rawX = device.getEncoderX();
        rawY = device.getEncoderY();
        localX = device.getPosX(DistanceUnit.MM);
        localY = device.getPosY(DistanceUnit.MM);
        podsValid = Maths.finite(localX, localY);
        if (!podsValid) reportFault("Pinpoint 位置非有限值");
        if (!podsValid || !hasPositionAnchor()) return;
        // Keep measured yaw available independently of the position check.
        if (Math.hypot(localX - previousX, localY - previousY) > 5000 * (now - previousTime) + 45) {
            reportFault("Pinpoint 位置跳变");
            podsValid = false;
            return;
        }
        double x = localX - originX, y = localY - originY;
        double c = Math.cos(headingShift), s = Math.sin(headingShift);
        forward = fieldForward + c * x - s * y;
        left = fieldLeft + s * x + c * y;
        copySample(now);
        positionValid = true;
    }

    public boolean anchor(double forwardMm, double rightMm, double fieldHeading) {
        if (!headingValid || Maths.time() - lastHeadingTime > HEADING_GRACE) return false;
        if (needsAnchor) {
            if (!podsValid) return false;
            try {
                configure();
            } catch (RuntimeException failure) {
                return false;
            }
        }
        forward = fieldForward = forwardMm;
        left = fieldLeft = -rightMm;
        originX = localX;
        originY = localY;
        headingShift = Maths.wrap(fieldHeading - sensorYaw);
        heading = Maths.wrap(fieldHeading);
        yawRate = lastYawRate;
        copySample(Maths.time());
        anchored = true;
        needsAnchor = false;
        positionAnchored = positionValid = podsValid;
        if (positionValid) status = DeviceStatus.READY.name();
        return true;
    }

    public boolean isAnchored() { return anchored && !needsAnchor; }
    public boolean hasPositionAnchor() { return positionAnchored && !needsAnchor; }
    public boolean requiresReanchor() { return needsAnchor; }
    public int encoderX() { return rawX; }
    public int encoderY() { return rawY; }
    public double sensorYaw() { return sensorYaw; }
    public boolean headingUsable() { return isAnchored() && Maths.time() - lastHeadingTime <= HEADING_GRACE; }
    public boolean recentlyLocated(double maxAge) {
        return hasPositionAnchor() && Maths.time() - previousTime <= maxAge;
    }

    private void reportFault(String reason) {
        status = lastFault = reason;
        faults++;
    }

    private void publishHeading(double now) {
        double age = now - lastHeadingTime;
        if (!needsAnchor && age <= HEADING_GRACE) {
            yawRate = lastYawRate;
            heading = Maths.wrap(sensorYaw + headingShift + (headingValid ? 0 : yawRate * age));
        } else yawRate = 0;
    }

    private void copySample(double now) {
        previousX = localX;
        previousY = localY;
        previousTime = now;
    }
}
