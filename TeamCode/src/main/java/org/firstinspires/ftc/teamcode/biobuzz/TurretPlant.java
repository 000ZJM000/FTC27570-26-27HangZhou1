package org.firstinspires.ftc.teamcode.biobuzz;

/** Per-direction duty versus speed model identified from short, open-loop pulses. */
public final class TurretPlant {
    private static final double[] DEFAULT = {
        0.09, 0.068, 0.11, 0.22, 0.05, 0.09, 0.068, 0.11, 0.22, 0.05, 0.85
    };
    private final double[] values;
    private final boolean identified;

    public TurretPlant(double... values) {
        identified = valid(values);
        this.values = (identified ? values : DEFAULT).clone();
    }

    public static TurretPlant load(CalibrationStore store) {
        return new TurretPlant(store.readExact(CalibrationStore.TURRET_PLANT, DEFAULT.length));
    }

    private static boolean valid(double[] values) {
        if (values == null || values.length != DEFAULT.length || !Maths.finite(values)) return false;
        for (int i : new int[]{0, 5}) {
            if (values[i] <= 0 || values[i] > 1 || values[i + 1] < 0 || values[i + 1] > 1
                    || values[i + 2] <= 0 || values[i + 3] <= 0 || values[i + 4] < 0) return false;
        }
        return values[10] > 0;
    }

    public boolean identified() { return identified; }
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
