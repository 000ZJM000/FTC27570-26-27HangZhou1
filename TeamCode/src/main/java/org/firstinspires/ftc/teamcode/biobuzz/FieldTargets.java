package org.firstinspires.ftc.teamcode.biobuzz;

public final class FieldTargets {
    public static final double FIELD_TILES = 6;
    private static final double GOAL_FORWARD = 2.5;
    private static final double LEFT_GOAL_RIGHT = 2;
    private static final double RIGHT_GOAL_RIGHT = 4;
    private static final double SPLIT_RIGHT = (LEFT_GOAL_RIGHT + RIGHT_GOAL_RIGHT) / 2;
    public final int goal;
    public final double bearing;
    public final double distanceMm;

    private FieldTargets(int goal, double bearing, double distanceMm) {
        this.goal = goal;
        this.bearing = bearing;
        this.distanceMm = distanceMm;
    }

    public static FieldTargets fromPose(double forwardMm, double leftMm, double heading) {
        double tile = RobotConfig.TILE_MM;
        int goal = -leftMm >= SPLIT_RIGHT * tile ? 2 : 1;
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double pivotForward = forwardMm + cos * RobotConfig.TURRET_FORWARD_MM + sin * RobotConfig.TURRET_RIGHT_MM;
        double pivotLeft = leftMm + sin * RobotConfig.TURRET_FORWARD_MM - cos * RobotConfig.TURRET_RIGHT_MM;
        double dx = GOAL_FORWARD * tile - pivotForward;
        double dy = -(goal == 1 ? LEFT_GOAL_RIGHT : RIGHT_GOAL_RIGHT) * tile - pivotLeft;
        return new FieldTargets(goal, Math.atan2(dy, dx), Math.hypot(dx, dy));
    }

    public static boolean inNoFireZone(double forwardMm, double leftMm) {
        double forward = forwardMm / RobotConfig.TILE_MM;
        double right = -leftMm / RobotConfig.TILE_MM;
        return right >= LEFT_GOAL_RIGHT && right <= RIGHT_GOAL_RIGHT
                && forward >= GOAL_FORWARD && forward <= FIELD_TILES - GOAL_FORWARD;
    }
}
