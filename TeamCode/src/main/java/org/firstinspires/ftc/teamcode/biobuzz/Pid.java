package org.firstinspires.ftc.teamcode.biobuzz;

public final class Pid {
    private final double p, i, d;
    private double integral;

    public Pid(double[] gains) {
        p = gains[0];
        i = gains[1];
        d = gains[2];
    }

    // Derivative comes from the measured velocity so a target step creates no derivative kick.
    public double update(double error, double measuredRate, double ff, double dt,
                         double minOutput, double maxOutput) {
        return update(error, measuredRate, ff, dt, minOutput, maxOutput, true);
    }

    public double update(double error, double measuredRate, double ff, double dt,
                         double minOutput, double maxOutput, boolean integrate) {
        double base = p * error - d * measuredRate + ff;
        double trial = integrate ? Maths.clip(integral + i * error * dt, -0.20, 0.20) : integral;
        double requested = base + trial;
        if ((requested <= maxOutput || trial <= integral)
                && (requested >= minOutput || trial >= integral)) {
            integral = trial;
        }
        return Maths.clip(base + integral, minOutput, maxOutput);
    }

    public void reset() { integral = 0; }
}
