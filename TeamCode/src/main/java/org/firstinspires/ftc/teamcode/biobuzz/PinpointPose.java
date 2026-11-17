package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.ErrorDetectionType;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.Register;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/** The field pose is a rigid transform of the Pinpoint's measured pose. */
public final class PinpointPose {
    private final GoBildaPinpointDriver device;
    private boolean anchored, positionAnchored, needsAnchor, podsValid;
    private double sensorYaw, headingShift, localX, localY, originX, originY, fieldForward, fieldLeft;
    private double previousTime, previousX, previousY, lastHeadingTime = Double.NEGATIVE_INFINITY;
    private double lastHeadingReadStart = Double.NEGATIVE_INFINITY;
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
        try {
            configure();
            device.resetPosAndIMU();
            anchored = positionAnchored = needsAnchor = false;
            headingShift = 0;
            lastHeadingTime = lastHeadingReadStart = Double.NEGATIVE_INFINITY;
            resetSince = Double.NaN;
            return true;
        } catch (RuntimeException failure) {
            status = "Pinpoint 初始化失败";
            return false;
        }
    }

    private void configure() {
        firmwareVersion = device.getDeviceVersion();
        if (firmwareVersion >= 3) {
            device.setErrorDetectionType(ErrorDetectionType.CRC);
            device.setBulkReadScope(Register.DEVICE_STATUS, Register.X_ENCODER_VALUE,
                    Register.Y_ENCODER_VALUE, Register.X_POSITION, Register.Y_POSITION, Register.H_ORIENTATION,
                    Register.H_VELOCITY);
        } else device.setErrorDetectionType(ErrorDetectionType.LOCAL_TEST);
        device.setOffsets(-RobotConfig.X_POD_RIGHT_MM, RobotConfig.Y_POD_FORWARD_MM, DistanceUnit.MM);
        device.setEncoderDirections(RobotConfig.X_POD_DIRECTION, RobotConfig.Y_POD_DIRECTION);
        device.setEncoderResolution(RobotConfig.POD_TICKS_PER_MM, DistanceUnit.MM);
    }

    public void update() {
        headingValid = podsValid = positionValid = false;
        if (device == null) {
            status = "找不到 " + RobotConfig.PINPOINT;
            return;
        }
        try {
            boolean badRead = false, headingJump = false;
            for (int attempt = 0; attempt < 2; attempt++) {
                double readStart = Maths.time();
                device.update();
                DeviceStatus state = device.getDeviceStatus();
                if (state == DeviceStatus.FAULT_BAD_READ) {
                    if (!badRead) {
                        badReads++;
                        reportFault(state.name());
                    }
                    badRead = true;
                    if (attempt == 1) badReadStreak++;
                    continue;
                }
                badReadStreak = 0;
                status = state.name();
                double now = Maths.time();
                if (state == DeviceStatus.CALIBRATING || state == DeviceStatus.NOT_READY) {
                    if (!Double.isFinite(resetSince)) resetSince = now;
                    if (anchored && now - resetSince >= 0.12) needsAnchor = true;
                } else resetSince = Double.NaN;
                boolean ready = state == DeviceStatus.READY;
                if (!ready && state != DeviceStatus.FAULT_X_POD_NOT_DETECTED
                        && state != DeviceStatus.FAULT_Y_POD_NOT_DETECTED
                        && state != DeviceStatus.FAULT_NO_PODS_DETECTED) {
                    if (anchored || state == DeviceStatus.FAULT_IMU_RUNAWAY) reportFault(state.name());
                    return;
                }
                // Compare snapshots within their I²C time windows and across the ±180° seam.
                double observed = device.getHeading(UnnormalizedAngleUnit.RADIANS);
                double rate = device.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
                now = Maths.time();
                if (!Double.isFinite(observed)) {
                    reportFault("Pinpoint 航向非有限值");
                    continue;
                }
                double nextYaw = Maths.wrap(observed);
                double elapsed = now - lastHeadingReadStart;
                double delta = Maths.wrap(nextYaw - sensorYaw);
                if (!needsAnchor && Math.abs(delta) > 0.13 + 18 * elapsed) {
                    if (!headingJump) {
                        headingJumps++;
                        lastJumpAngle = delta;
                        lastJumpSeconds = elapsed;
                        lastJumpRate = rate;
                        reportFault("Pinpoint 航向跳变");
                    } else status = "Pinpoint 航向跳变";
                    headingJump = true;
                    continue;
                }
                if (headingJump) recoveredHeadingJumps++;
                if (badRead && ready) recoveredReads++;
                sensorYaw = nextYaw;
                lastHeadingReadStart = readStart;
                lastHeadingTime = now;
                yawRate = Double.isFinite(rate) && Math.abs(rate) <= 20 ? rate : 0;
                headingValid = true;
                heading = Maths.wrap(sensorYaw + headingShift);
                if (needsAnchor) status = "Pinpoint 重新初始化，回起点按 Y";
                if (ready) updatePosition(now);
                else reportFault(state.name());
                return;
            }
        } catch (RuntimeException failure) {
            podsValid = positionValid = false;
            reportFault("Pinpoint 读取失败 " + failure.getClass().getSimpleName());
        }
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
        if (!headingValid) return false;
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
        copySample(Maths.time());
        anchored = true;
        needsAnchor = false;
        positionAnchored = positionValid = podsValid;
        return true;
    }

    public boolean isAnchored() { return anchored && !needsAnchor; }
    public boolean hasPositionAnchor() { return positionAnchored && !needsAnchor; }
    public boolean requiresReanchor() { return needsAnchor; }
    public int encoderX() { return rawX; }
    public int encoderY() { return rawY; }
    public double sensorYaw() { return sensorYaw; }
    public boolean headingUsable() { return isAnchored() && Maths.time() - lastHeadingTime <= 0.12; }
    public boolean recentlyLocated(double maxAge) {
        return hasPositionAnchor() && Maths.time() - previousTime <= maxAge;
    }

    private void reportFault(String reason) {
        status = lastFault = reason;
        faults++;
        if (!headingValid) yawRate = 0;
    }

    private void copySample(double now) {
        previousX = localX;
        previousY = localY;
        previousTime = now;
    }
}
