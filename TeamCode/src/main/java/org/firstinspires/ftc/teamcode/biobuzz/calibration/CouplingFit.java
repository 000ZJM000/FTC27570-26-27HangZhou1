package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.Maths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Robust motor velocity and acceleration fit after subtracting the measured local friction. */
final class CouplingFit {
    private static final double MAX_KV = 1.2, MAX_KA = 0.20;
    private final List<double[]> windows = new ArrayList<>();

    void add(double velocity, double acceleration, double power) {
        if (velocity > 0.12 && Maths.finite(velocity, acceleration, power)) {
            windows.add(new double[]{velocity, acceleration, power});
        }
    }

    int size() { return windows.size(); }

    double[] fit() {
        if (windows.size() < 12) return null;
        double[] speeds = windows.stream().mapToDouble(row -> row[0]).sorted().toArray();
        if (speeds[speeds.length * 9 / 10] - speeds[speeds.length / 10] < 0.40) return null;
        double[] weights = new double[windows.size()];
        double[] residuals = new double[weights.length];
        Arrays.fill(weights, 1);
        double kv = 0, ka = 0;
        for (int iteration = 0; iteration < 16; iteration++) {
            double vv = 0, aa = 0, va = 0, vp = 0, ap = 0;
            for (int i = 0; i < windows.size(); i++) {
                double[] row = windows.get(i);
                double v = row[0], a = row[1], p = row[2], weight = weights[i];
                vv += weight * v * v;
                aa += weight * a * a;
                va += weight * v * a;
                vp += weight * v * p;
                ap += weight * a * p;
            }
            if (vv < 1e-7) return null;
            if (aa < 1e-7 || vv * aa - va * va < 0.04 * vv * aa) {
                // Steady turns can still identify kV, even when accelerations cannot identify kA.
                kv = Maths.clip(vp / vv, 0, MAX_KV);
                ka = 0;
            } else {
                for (int sweep = 0; sweep < 40; sweep++) {
                    kv = Maths.clip((vp - va * ka) / vv, 0, MAX_KV);
                    ka = Maths.clip((ap - va * kv) / aa, 0, MAX_KA);
                }
            }
            for (int i = 0; i < residuals.length; i++) {
                double[] row = windows.get(i);
                residuals[i] = Math.abs(row[2] - row[0] * kv - row[1] * ka);
            }
            double cutoff = Math.max(0.022, 1.7 * Maths.median(residuals, residuals.length));
            for (int i = 0; i < weights.length; i++) {
                weights[i] = Math.min(1, cutoff / Math.max(1e-9, residuals[i]));
            }
        }
        return kv > 0.002 ? new double[]{kv, ka} : null;
    }
}
