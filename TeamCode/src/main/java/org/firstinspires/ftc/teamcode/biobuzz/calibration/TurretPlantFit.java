package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.Maths;

import java.util.ArrayList;
import java.util.List;

/** Fits a delayed first-order speed response jointly across several constant-duty pulses. */
final class TurretPlantFit {
    private final List<double[]> points = new ArrayList<>();

    void add(double t, double duty, double speed) {
        if (Maths.finite(t, duty, speed)) points.add(new double[]{t, duty, Math.max(0, speed)});
    }

    int count() { return points.size(); }

    double[] fit(double breakaway) {
        if (points.size() < 22) return null;
        double min = 1, max = 0, peak = 0;
        for (double[] p : points) {
            min = Math.min(min, p[1]);
            max = Math.max(max, p[1]);
            peak = Math.max(peak, p[2]);
        }
        if (max - min < 0.065 || peak < 0.30) return null;
        double best = Double.POSITIVE_INFINITY;
        double[] model = null;
        double[] shape = new double[points.size()];
        double cut = Math.max(0.10, 0.12 * peak);
        for (int i = 0; i < 34; i++) {
            double tau = 0.025 * Math.pow(0.95 / 0.025, i / 33.0);
            for (int j = 0; j <= 12; j++) {
                double delay = j * 0.0125;
                double gain = 1 / 0.12, drag = breakaway * 0.75;
                for (int n = 0; n < points.size(); n++) {
                    shape[n] = -Math.expm1(-Math.max(0, points.get(n)[0] - delay) / tau);
                }
                for (int it = 0; it < 4; it++) {
                    double xx = 0, xz = 0, zz = 0, xy = 0, zy = 0;
                    for (int n = 0; n < points.size(); n++) {
                        double[] p = points.get(n);
                        double z = shape[n], x = z * p[1];
                        double residual = Math.abs(p[2] - gain * (x - z * drag));
                        double weight = it == 0 ? 1 : Math.min(1, cut / Math.max(1e-7, residual));
                        xx += weight * x * x;
                        xz += weight * x * z;
                        zz += weight * z * z;
                        xy += weight * x * p[2];
                        zy += weight * z * p[2];
                    }
                    double det = xx * zz - xz * xz;
                    if (det < 1e-7) break;
                    gain = (xy * zz - zy * xz) / det;
                    if (gain < 1e-5) break;
                    drag = Maths.clip((gain * xz - zy) / (gain * zz), 0.005, breakaway + 0.045);
                    gain = Maths.clip((xy - drag * zy) / (xx - 2 * drag * xz + drag * drag * zz), 1.5, 65);
                }
                double error = 0;
                for (int n = 0; n < points.size(); n++) {
                    double[] p = points.get(n);
                    double r = Math.abs(p[2] - gain * Math.max(0, p[1] - drag) * shape[n]);
                    error += r < cut ? 0.5 * r * r : cut * (r - 0.5 * cut);
                }
                error /= points.size();
                if (error < best) {
                    best = error;
                    model = new double[]{breakaway, drag, 1 / gain, tau, delay};
                }
            }
        }
        return best < 0.085 * peak * peak ? model : null;
    }
}
