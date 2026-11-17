package org.firstinspires.ftc.teamcode.biobuzz;

/** Limits the angle setpoint's speed and acceleration during a rewind. */
final class TurretRewind {
    private static final double MAX_RATE = Math.toRadians(185);
    private static final double ACCELERATION = Math.toRadians(340);
    private boolean active;
    double angle, rate;

    void reset() { active = false; }

    void update(double current, double speed, double destination, double destinationRate, double dt) {
        if (!active) {
            angle = current;
            rate = Maths.clip(speed, -MAX_RATE, MAX_RATE);
            active = true;
        }
        double error = destination - angle;
        double approach = Math.copySign(Math.sqrt(2 * ACCELERATION * Math.abs(error)), error);
        double requestedRate = Maths.clip(approach + destinationRate, -MAX_RATE, MAX_RATE);
        double nextRate = Maths.clip(requestedRate, rate - ACCELERATION * dt, rate + ACCELERATION * dt);
        double nextAngle = angle + (rate + nextRate) * dt * 0.5;
        if (error * (destination - nextAngle) <= 0 && Math.abs(nextRate - destinationRate) <= ACCELERATION * dt) {
            angle = destination;
            rate = destinationRate;
        } else {
            angle = nextAngle;
            rate = nextRate;
        }
    }

    boolean arrived(double current, double speed, double destination, double destinationRate) {
        double tolerance = Math.toRadians(2.5);
        return Math.abs(destination - angle) <= tolerance && Math.abs(destination - current) <= tolerance
                && Math.abs(speed - destinationRate) < 0.32;
    }
}
