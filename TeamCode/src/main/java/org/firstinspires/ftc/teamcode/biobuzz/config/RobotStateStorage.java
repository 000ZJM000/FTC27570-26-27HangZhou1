package org.firstinspires.ftc.teamcode.biobuzz.config;

/**
 * Shared cross-OpMode handoff state between Autonomous and TeleOp.
 * Keeps field pose and turret right-stop encoder tick across the 8-second transition without touching the robot.
 */
public final class RobotStateStorage {
    public static volatile boolean autoPoseValid = false;
    public static volatile double autoForwardMm = 0.0;
    public static volatile double autoLeftMm = 0.0;
    public static volatile double autoHeadingRad = 0.0;

    public static volatile boolean turretHomeValid = false;
    public static volatile int turretHomeTick = 0;
    public static volatile boolean turretLastSampleValid = false;
    public static volatile int turretLastTick = 0;
    public static volatile double turretLastAngleRad = 0.0;

    private RobotStateStorage() {}

    public static void clear() {
        clearAutoPose();
        clearTurretHome();
    }

    public static void clearAutoPose() {
        autoPoseValid = false;
        autoForwardMm = 0.0;
        autoLeftMm = 0.0;
        autoHeadingRad = 0.0;
    }

    public static void clearTurretHome() {
        turretHomeValid = false;
        turretHomeTick = 0;
        turretLastSampleValid = false;
        turretLastTick = 0;
        turretLastAngleRad = 0.0;
    }

    public static void recordAutoPose(double forwardMm, double leftMm, double headingRad) {
        if (!Double.isFinite(forwardMm) || !Double.isFinite(leftMm) || !Double.isFinite(headingRad)) return;
        autoForwardMm = forwardMm;
        autoLeftMm = leftMm;
        autoHeadingRad = headingRad;
        autoPoseValid = true;
    }

    public static void recordTurretHome(int homeTick) {
        turretHomeTick = homeTick;
        turretHomeValid = true;
    }

    public static void recordTurretSample(int currentTick, double angleRad) {
        if (!Double.isFinite(angleRad)) return;
        turretLastTick = currentTick;
        turretLastAngleRad = angleRad;
        turretLastSampleValid = true;
    }

    public static boolean hasAutoPose() { return autoPoseValid; }

    public static boolean hasTurretHome() { return turretHomeValid; }

    public static boolean hasTurretLastSample() { return turretLastSampleValid; }

    public static boolean autoPoseMovedFromOrigin() {
        return autoPoseValid && (Math.hypot(autoForwardMm, autoLeftMm) > 25.0 || Math.abs(autoHeadingRad) > Math.toRadians(2.0));
    }
}
