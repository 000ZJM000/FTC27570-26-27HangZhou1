package org.firstinspires.ftc.teamcode.biobuzz;

public final class ShooterProfile {
    private ShooterProfile() {}

    public static double targetRpm(int goal, double rangeMm) {
        // Add a distance-to-RPM table using rangeMm once the hood and firing geometry are finalized.
        double rpm = goal == 2 ? RobotConfig.GOAL_TWO_RPM : RobotConfig.GOAL_ONE_RPM;
        return rpm > 0 ? rpm : RobotConfig.FLYWHEEL_SHOOT_RPM;
    }
}
