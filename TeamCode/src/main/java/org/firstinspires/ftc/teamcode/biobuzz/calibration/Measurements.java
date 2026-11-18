package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import org.firstinspires.ftc.teamcode.biobuzz.Maths;
import java.util.ArrayList;
import java.util.List;

final class Measurements {
    private final double[] values = new double[128];
    private int count;

    void add(double value) {
        if (Double.isFinite(value) && count < values.length) values[count++] = value;
    }

    double median() { return Maths.median(values, count); }
    int size() { return count; }

    double relativeSpread() {
        double middle = median();
        double[] differences = new double[count];
        for (int i = 0; i < count; i++) differences[i] = Math.abs(values[i] - middle);
        return Maths.median(differences, count) / Math.max(1e-9, Math.abs(middle));
    }

    static final class PowerFit {
        private final List<double[]> points = new ArrayList<>();

        void add(double speed, double power) {
            if (speed > 0 && Maths.finite(speed, power)) points.add(new double[]{speed, power});
        }

        // Theil–Sen line. Median pair slopes damp out bad passes and battery sag.
        double[] fit(double minimumSpeedDifference) {
            Measurements slopes = new Measurements();
            for (int i = 0; i < points.size(); i++) {
                for (int j = i + 1; j < points.size(); j++) {
                    double[] a = points.get(i), b = points.get(j);
                    double dx = b[0] - a[0];
                    if (Math.abs(dx) > minimumSpeedDifference) slopes.add((b[1] - a[1]) / dx);
                }
            }
            double slope = slopes.median();
            if (slopes.size() < 2 || !(slope > 0)) return null;
            Measurements intercepts = new Measurements();
            for (double[] point : points) intercepts.add(point[1] - slope * point[0]);
            return new double[]{Math.max(0, intercepts.median()), slope};
        }

    }
}
