package org.firstinspires.ftc.teamcode.biobuzz;

/** Anticipate the motor's stopping travel using its identified response and current closing rate. */
final class TurretBrake {
    private static final double MAX_DUTY = 0.38;

    private TurretBrake() {}

    static double duty(double error, double relativeRate, double frictionDuty, double appliedDuty,
                       double kV, double tau, double delay) {
        if (Math.abs(relativeRate) < 0.20 || (error * relativeRate < 0
                && Math.abs(error) > Math.toRadians(2))) return 0;
        double speed = Math.abs(relativeRate);
        double reaction = Maths.clip(delay + 0.025 + Math.max(0, Math.copySign(1, relativeRate)
                * appliedDuty) / 8, 0.025, 0.23);
        double deceleration = (0.16 + 0.65 * kV * speed) / Math.max(0.004, kV * tau);
        double stopping = reaction * speed + speed * speed / (2 * deceleration);
        double blend = Maths.clip((stopping - Math.abs(error)) / Math.max(0.02, 0.72 * stopping), 0, 1);
        // Ease the contribution in. Friction and the derivative term still supply their own damping.
        double weight = blend * blend * (3 - 2 * blend) * Maths.clip((speed - 0.20) / 0.36, 0, 1);
        double brake = Math.min(MAX_DUTY, 0.055 + 0.65 * Math.abs(frictionDuty) + 0.78 * kV * speed);
        return -Math.copySign(weight * brake, relativeRate);
    }
}
