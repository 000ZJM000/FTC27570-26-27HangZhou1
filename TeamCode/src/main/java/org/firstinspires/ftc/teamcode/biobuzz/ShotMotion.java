package org.firstinspires.ftc.teamcode.biobuzz;

/** Field coordinates. Right and forward are positive, and all position and velocity values use millimeters. */
final class ShotMotion {
    private final double right, forward, rightRate, forwardRate;
    private final double cos, sin, rightOrbitRate, forwardOrbitRate;

    ShotMotion(PinpointPose pose) {
        cos = Math.cos(pose.heading);
        sin = Math.sin(pose.heading);
        double orbitForward = cos * RobotConfig.TURRET_FORWARD_MM + sin * RobotConfig.TURRET_RIGHT_MM;
        double orbitRight = -sin * RobotConfig.TURRET_FORWARD_MM + cos * RobotConfig.TURRET_RIGHT_MM;
        forward = pose.forward + orbitForward;
        right = -pose.left + orbitRight;
        rightRate = pose.positionValid ? -pose.leftVelocity : 0;
        forwardRate = pose.positionValid ? pose.forwardVelocity : 0;
        rightOrbitRate = -orbitForward * pose.yawRate;
        forwardOrbitRate = orbitRight * pose.yawRate;
    }

    ShotVectorField.Vector predict(CalibratedShotMap map, boolean rightHalf, boolean braking) {
        double tile = RobotConfig.TILE_MM;
        double range = Math.hypot(right - tile * map.goalRight(rightHalf), forward - tile * map.goalForward());
        double flight = flightTime(range);
        double delay = RobotConfig.SHOT_RELEASE_SECONDS;
        double vf = cos * forwardRate - sin * rightRate;
        double vr = sin * forwardRate + cos * rightRate;
        double along = lead(vf, braking ? RobotConfig.BRAKE_FORWARD_MM_S2 : 0, delay, flight);
        double across = lead(vr, braking ? RobotConfig.BRAKE_SIDE_MM_S2 : 0, delay, flight);
        double virtualRight = right - sin * along + cos * across + rightOrbitRate * (delay + flight);
        double virtualForward = forward + cos * along + sin * across + forwardOrbitRate * (delay + flight);
        return map.sample(virtualRight / tile, virtualForward / tile, rightHalf);
    }

    double lineRate(double goalRightTiles, double goalForwardTiles) {
        return CalibratedShotMap.lineRate(right, forward, rightRate + rightOrbitRate,
                forwardRate + forwardOrbitRate, goalRightTiles * RobotConfig.TILE_MM,
                goalForwardTiles * RobotConfig.TILE_MM);
    }

    static double flightTime(double distanceMm) {
        // Initial near/far timing estimates. Replace with measured flight times from the field.
        double fraction = (distanceMm - RobotConfig.FLIGHT_NEAR_MM)
                / (RobotConfig.FLIGHT_FAR_MM - RobotConfig.FLIGHT_NEAR_MM);
        return RobotConfig.FLIGHT_NEAR_SECONDS
                + (RobotConfig.FLIGHT_FAR_SECONDS - RobotConfig.FLIGHT_NEAR_SECONDS) * Maths.clip(fraction, 0, 1);
    }

    static double lead(double speed, double deceleration, double delay, double flight) {
        double magnitude = Math.abs(speed);
        double moving = deceleration > 0 ? Math.min(delay, magnitude / deceleration) : delay;
        double release = Math.max(0, magnitude - deceleration * moving);
        return Math.copySign(0.5 * (magnitude + release) * moving + release * flight, speed);
    }
}
