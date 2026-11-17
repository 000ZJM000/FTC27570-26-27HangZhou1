package org.firstinspires.ftc.teamcode.biobuzz;

/** Per-direction duty versus speed model identified from short, open-loop pulses. */
public final class TurretPlant {
    private final double[] values;

    public TurretPlant(double... values) { this.values = values.clone(); }

    public static TurretPlant load(CalibrationStore store) {
        return new TurretPlant(store.read(CalibrationStore.TURRET_PLANT,
                0.09, 0.068, 0.11, 0.22, 0.05, 0.09, 0.068, 0.11, 0.22, 0.05, 0.85));
    }

    private int side(int direction) { return direction > 0 ? 0 : 5; }
    public double breakaway(int direction) { return values[side(direction)]; }
    public double rolling(int direction) { return values[side(direction) + 1]; }
    public double kV(int direction) { return values[side(direction) + 2]; }
    public double timeConstant(int direction) { return values[side(direction) + 3]; }
    public double delay(int direction) { return values[side(direction) + 4]; }
    public double baseline() { return values[10]; }
    public double[] values() { return values.clone(); }

    public double[] pidSeed(double bandwidthScale) {
        double tau = Math.max(timeConstant(1), timeConstant(-1));
        double kV = (kV(1) + kV(-1)) * 0.5;
        double delay = Math.max(delay(1), delay(-1));
        double slowPole = Math.min(4.8, Math.min(0.56 / (0.095 + delay), 0.88 / tau + 0.55));
        double rate = slowPole * Maths.clip(bandwidthScale, 0.48, 1.25);
        double fastPole = Math.max(2.15 * rate, 1 / tau - rate);
        double p = tau * kV * rate * fastPole;
        double d = Math.max(0, tau * kV * (rate + fastPole) - kV);
        return new double[]{p, Maths.clip(0.13 * p, 0.012, 0.22), d};
    }
}
