package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.Arrays;
import java.util.Locale;

/**
 * Per-direction inverse mapping from normalized turn joystick [-1, 1] to 12 V motor turn command,
 * compensating drivetrain breakaway deadzone and high-speed saturation so steady yaw rate scales
 * linearly with right-stick deflection.
 *
 * Stored array layout (11 doubles):
 * [0..3] left  kS, c1, c2, c3  | [4] measured left  max rate (rad/s)
 * [5..8] right kS, c1, c2, c3  | [9] measured right max rate (rad/s)
 * [10]   symmetric target max rate omegaMax (rad/s)
 */
public final class TurnLinearity {
    private static final int LENGTH = 11;
    private static final double MIN_RATE_SLOPE = 0.02;
    private final double[] values;
    private final boolean calibrated;

    private TurnLinearity(double[] values) {
        calibrated = valid(values);
        this.values = calibrated ? values.clone() : new double[LENGTH];
    }

    public static TurnLinearity load(CalibrationStore store) {
        return new TurnLinearity(store.readExact(CalibrationStore.TURN_LINEARITY, LENGTH));
    }

    public static boolean valid(double[] v) {
        if (v == null || v.length != LENGTH || !Maths.finite(v)) return false;
        if (v[10] < 0.4 || v[10] > 18) return false;
        for (int offset : new int[]{0, 5}) {
            double ks = v[offset], c1 = v[offset + 1], c2 = v[offset + 2], c3 = v[offset + 3];
            double maxRate = v[offset + 4];
            if (ks < 0.02 || ks > 0.45 || maxRate < v[10] * 0.95) return false;
            double endDuty = ks + c1 + c2 + c3;
            if (endDuty < ks + 0.15 || endDuty > 1.35) return false;
            for (int i = 0; i <= 4; i++) {
                double s = i * 0.25;
                double derivative = c1 + 2 * c2 * s + 3 * c3 * s * s;
                if (derivative < 0.02) return false;
            }
        }
        return true;
    }

    public boolean calibrated() { return calibrated; }
    public double omegaMax() { return calibrated ? values[10] : 0; }
    public double maxRate(int direction) { return calibrated ? values[direction >= 0 ? 4 : 9] : 0; }
    public double breakaway(int direction) { return calibrated ? values[direction >= 0 ? 0 : 5] : 0; }

    public double expectedRate(double stick) {
        return calibrated ? Maths.clip(stick, -1, 1) * values[10] : 0;
    }

    public double[] values() { return calibrated ? values.clone() : null; }

    public double mapNominal(double stick, double forward, double left) {
        if (stick == 0 || !Double.isFinite(stick)) return 0;
        if (!calibrated) return Maths.clip(stick, -1, 1);
        int base = stick > 0 ? 0 : 5;
        double s = Maths.clip(Math.abs(stick), 0, 1);
        double trans = Maths.clip((Math.abs(forward) + Math.abs(left)) / 0.35, 0, 1);
        double ks = values[base] * (1 - 0.55 * trans);
        double c1 = values[base + 1], c2 = values[base + 2], c3 = values[base + 3];
        double nominal = Maths.clip(ks + s * (c1 + s * (c2 + s * c3)), 0, 1.25);
        return Math.copySign(nominal, stick);
    }

    public double mapDuty(double stick, double forward, double left, BatteryVoltage battery, double maxPower) {
        if (stick == 0 || !Double.isFinite(stick)) return 0;
        if (!calibrated) return Maths.clip(stick, -maxPower, maxPower);
        double nominal = mapNominal(stick, forward, left);
        return battery.duty(nominal, maxPower);
    }

    public String equation(int direction) {
        if (!calibrated) return "未校准";
        int base = direction >= 0 ? 0 : 5;
        return String.format(Locale.ROOT,
                "d(u) = %.3f %+.3f·u %+.3f·u² %+.3f·u³ (峰值 %.0f°/s)",
                values[base], values[base + 1], values[base + 2], values[base + 3],
                Math.toDegrees(values[base + 4]));
    }

    public String summary() {
        return calibrated
                ? String.format(Locale.ROOT, "已校准 ω_max=%.0f°/s (kS 左%.2f/右%.2f)",
                Math.toDegrees(values[10]), values[0], values[5])
                : "未校准（默认线性直通）";
    }

    public static TurnLinearity fit(double ksLeft, double[] dutiesLeft, double[] ratesLeft,
                                    double ksRight, double[] dutiesRight, double[] ratesRight) {
        if (dutiesLeft == null || ratesLeft == null || dutiesRight == null || ratesRight == null) return null;
        if (dutiesLeft.length != ratesLeft.length || dutiesRight.length != ratesRight.length) return null;
        if (dutiesLeft.length < 4 || dutiesRight.length < 4) return null;

        double[] monoLeft = enforceMonotonicRates(dutiesLeft, ratesLeft);
        double[] monoRight = enforceMonotonicRates(dutiesRight, ratesRight);
        if (monoLeft == null || monoRight == null) return null;

        double maxRateLeft = monoLeft[monoLeft.length - 1];
        double maxRateRight = monoRight[monoRight.length - 1];
        double omegaMax = Math.min(maxRateLeft, maxRateRight);
        if (omegaMax < 0.6) return null;

        double[] leftSide = fitSide(ksLeft, dutiesLeft, monoLeft, omegaMax);
        double[] rightSide = fitSide(ksRight, dutiesRight, monoRight, omegaMax);
        if (leftSide == null || rightSide == null) return null;

        double[] model = new double[LENGTH];
        System.arraycopy(leftSide, 0, model, 0, 4);
        model[4] = maxRateLeft;
        System.arraycopy(rightSide, 0, model, 5, 4);
        model[9] = maxRateRight;
        model[10] = omegaMax;
        return valid(model) ? new TurnLinearity(model) : null;
    }

    private static double[] enforceMonotonicRates(double[] duties, double[] rates) {
        double[] mono = Arrays.copyOf(rates, rates.length);
        for (int i = 0; i < mono.length; i++) {
            if (!Maths.finite(duties[i], mono[i]) || duties[i] <= 0) return null;
            if (mono[i] < 0) mono[i] = 0;
            if (i > 0) {
                mono[i] = Math.max(mono[i], mono[i - 1] + MIN_RATE_SLOPE * Math.max(0, duties[i] - duties[i - 1]));
            }
        }
        return mono;
    }

    private static double[] fitSide(double ks, double[] duties, double[] rates, double omegaMax) {
        double safeKs = Maths.clip(ks, 0.03, Math.min(0.40, duties[0] - 0.01));
        double endpointDuty = interpolateDutyAt(duties, rates, omegaMax);
        double linearSlope = Math.max(0.25, endpointDuty - safeKs);
        double[] prior = {Math.max(0.25, 0.92 - safeKs), 0, 0};
        LeastSquares solver = new LeastSquares(new double[]{0.002, 0.020, 0.060}, prior);
        for (int i = 0; i < duties.length; i++) {
            double s = Maths.clip(rates[i] / omegaMax, 0, 1.05);
            double s2 = s * s;
            solver.add(new double[]{s, s2, s2 * s}, duties[i] - safeKs, 1.0);
        }
        solver.add(new double[]{1.0, 1.0, 1.0}, endpointDuty - safeKs, 2.0);
        double[] solved = solver.solve();
        double c1 = solved != null ? solved[0] : linearSlope;
        double c2 = solved != null ? solved[1] : 0;
        double c3 = solved != null ? solved[2] : 0;

        double minRequired = Math.max(0.03, 0.12 * linearSlope);
        if (minDerivative(c1, c2, c3) < minRequired) {
            double low = 0, high = 1;
            for (int iter = 0; iter < 24; iter++) {
                double alpha = 0.5 * (low + high);
                double tc1 = linearSlope + alpha * (c1 - linearSlope);
                double tc2 = alpha * c2;
                double tc3 = alpha * c3;
                if (minDerivative(tc1, tc2, tc3) >= minRequired) low = alpha;
                else high = alpha;
            }
            c1 = linearSlope + low * (c1 - linearSlope);
            c2 = low * c2;
            c3 = low * c3;
        }
        return new double[]{safeKs, c1, c2, c3};
    }

    private static double interpolateDutyAt(double[] duties, double[] rates, double targetRate) {
        if (targetRate <= rates[0]) return duties[0];
        for (int i = 1; i < rates.length; i++) {
            if (targetRate <= rates[i]) {
                double span = rates[i] - rates[i - 1];
                double fraction = span > 1e-6 ? (targetRate - rates[i - 1]) / span : 1.0;
                return duties[i - 1] + fraction * (duties[i] - duties[i - 1]);
            }
        }
        return duties[duties.length - 1];
    }

    private static double minDerivative(double c1, double c2, double c3) {
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i <= 20; i++) {
            double s = i / 20.0;
            double d = c1 + 2 * c2 * s + 3 * c3 * s * s;
            if (d < min) min = d;
        }
        return min;
    }
}
