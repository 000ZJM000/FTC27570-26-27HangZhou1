package org.firstinspires.ftc.teamcode.biobuzz;

/** Keeps one RT request active while the chassis moves through changing conditions. */
final class RunShotSequence {
    private static final double READY_SECONDS = 0.06;
    private static final double HOLD_AIM_TOLERANCE = Math.toRadians(4);
    private static final double HOLD_RPM_TOLERANCE = 0.15;
    private boolean feeding;
    private double readySince = Double.NaN;

    double aimTolerance() { return feeding ? HOLD_AIM_TOLERANCE : RobotConfig.SHOT_AIM_TOLERANCE; }
    double rpmTolerance() { return feeding ? HOLD_RPM_TOLERANCE : RobotConfig.SHOT_RPM_TOLERANCE; }

    boolean update(double now, boolean requested, boolean blocked, boolean aimReady, boolean rpmReady) {
        if (!requested || blocked || !aimReady || !rpmReady) {
            feeding = false;
            readySince = Double.NaN;
        } else {
            if (!Double.isFinite(readySince)) readySince = now;
            feeding = now - readySince >= READY_SECONDS;
        }
        return feeding;
    }
}
