package org.firstinspires.ftc.teamcode.biobuzz;

final class CornerAim {
    final double bearing, rpm;
    private final int goal;
    private final double lineRate;

    private CornerAim(int goal, ShotVectorField.Vector shot, double lineRate) {
        this.goal = goal;
        bearing = shot.bearing();
        rpm = shot.rpm();
        this.lineRate = lineRate;
    }

    static CornerAim fromPose(CalibratedShotMap map, PinpointPose pose, boolean braking) {
        if (map == null || !map.available() || !pose.headingUsable()
                || !positionForAim(pose)) return null;
        boolean rightHalf = -pose.left >= FieldTargets.FIELD_TILES * RobotConfig.TILE_MM / 2;
        ShotMotion motion = new ShotMotion(pose);
        ShotVectorField.Vector shot = motion.predict(map, rightHalf, braking);
        if (shot == null) return null;
        return new CornerAim(rightHalf ? 4 : 3, shot, motion.lineRate(map.goalRight(rightHalf), map.goalForward()));
    }

    static double targetRpm(CornerAim aim) {
        return aim == null ? RobotConfig.FLYWHEEL_TARGET_RPM : aim.rpm;
    }

    static boolean positionForAim(PinpointPose pose) {
        return pose.positionValid || pose.recentlyLocated(0.40);
    }

    static boolean noFireZone(PinpointPose pose) {
        return pose.positionValid && pose.headingValid
                && FieldTargets.fullyInNoFireZone(pose.forward, pose.left, pose.heading);
    }

    static void track(Turret turret, PinpointPose pose, CornerAim aim, double yawTrim, double turn, double dt) {
        if (aim == null) turret.aim(pose, 0, 0, 0, yawTrim, turn, dt);
        else turret.aim(pose, aim.goal, aim.bearing, aim.lineRate, yawTrim, turn, dt);
    }
}
