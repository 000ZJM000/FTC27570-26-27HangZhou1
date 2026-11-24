package org.firstinspires.ftc.teamcode.biobuzz.core;

import java.util.Arrays;

/** Core scalar, angle, time, PID, and regularized least-squares math utilities. */
public final class Maths {
    public static final double TURN = 2 * Math.PI;

    private Maths() {}

    public static double clip(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    public static double wrap(double angle) {
        double wrapped = angle % TURN;
        if (wrapped >= Math.PI) return wrapped - TURN;
        return wrapped < -Math.PI ? wrapped + TURN : wrapped;
    }

    public static double deadband(double command) {
        double magnitude = Math.abs(command);
        return magnitude < 0.06 ? 0 : Math.copySign((magnitude - 0.06) / 0.94, command);
    }

    public static double median(double[] values, int count) {
        if (count == 0) return Double.NaN;
        double[] ordered = Arrays.copyOf(values, count);
        Arrays.sort(ordered);
        return (ordered[(count - 1) / 2] + ordered[count / 2]) / 2;
    }

    public static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }

    public static double time() {
        return System.nanoTime() * 1e-9;
    }

    /** Anti-windup PIDF controller with derivative-on-measurement. */
    public static final class Pid {
        private final double p, i, d;
        private double integral;

        public Pid(double[] gains) {
            p = gains[0];
            i = gains[1];
            d = gains[2];
        }

        public double update(double error, double measuredRate, double ff, double dt,
                             double minOutput, double maxOutput) {
            return update(error, measuredRate, ff, dt, minOutput, maxOutput, true);
        }

        public double update(double error, double measuredRate, double ff, double dt,
                             double minOutput, double maxOutput, boolean integrate) {
            return update(error, measuredRate, ff, dt, minOutput, maxOutput, integrate, 0);
        }

        public double update(double error, double measuredRate, double ff, double dt,
                             double minOutput, double maxOutput, boolean integrate, double minimumIntegralGain) {
            double base = p * error - d * measuredRate + ff;
            double trial = integrate
                    ? clip(integral + Math.max(i, minimumIntegralGain) * error * dt, -0.20, 0.20) : integral;
            double requested = base + trial;
            if ((requested <= maxOutput || trial <= integral)
                    && (requested >= minOutput || trial >= integral)) {
                integral = trial;
            }
            return clip(base + integral, minOutput, maxOutput);
        }

        public double integralDuty() { return integral; }

        public void reset() { integral = 0; }
    }

    /** Small weighted, ridge-regularized least-squares fit. */
    public static final class LeastSquares {
        private final double[][] rows;

        public LeastSquares(double[] penalty, double[] prior) {
            rows = new double[penalty.length][penalty.length + 1];
            for (int i = 0; i < rows.length; i++) {
                rows[i][i] = penalty[i];
                if (prior != null) rows[i][rows.length] = penalty[i] * prior[i];
            }
        }

        public void add(double[] features, double target, double weight) {
            for (int i = 0; i < rows.length; i++) {
                double factor = weight * features[i];
                for (int j = 0; j < rows.length; j++) rows[i][j] += factor * features[j];
                rows[i][rows.length] += factor * target;
            }
        }

        public double[] solve() {
            int size = rows.length;
            for (int column = 0; column < size; column++) {
                int pivot = column;
                for (int row = column + 1; row < size; row++) {
                    if (Math.abs(rows[row][column]) > Math.abs(rows[pivot][column])) pivot = row;
                }
                if (Math.abs(rows[pivot][column]) < 1e-12) return null;
                double[] swapped = rows[column];
                rows[column] = rows[pivot];
                rows[pivot] = swapped;
                double divisor = rows[column][column];
                for (int j = column; j <= size; j++) rows[column][j] /= divisor;
                for (int row = 0; row < size; row++) {
                    if (row == column) continue;
                    double factor = rows[row][column];
                    for (int j = column; j <= size; j++) rows[row][j] -= factor * rows[column][j];
                }
            }
            double[] result = new double[size];
            for (int i = 0; i < size; i++) result[i] = rows[i][size];
            return result;
        }
    }
}
