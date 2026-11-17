package org.firstinspires.ftc.teamcode.biobuzz.calibration;

final class StepScore {
    private final double duration;
    private double errorArea, overshoot, tailTime, tailError, tailSquare, effort, lastPower;
    private double lastSample, withinToleranceSince = Double.NaN;

    StepScore(double duration) { this.duration = duration; }

    // Each response is scaled by the step size to score different magnitudes in the same units.
    void sample(double time, double dt, double progress, double power) {
        double error = 1 - progress;
        errorArea += Math.abs(error) * dt;
        overshoot = Math.max(overshoot, -error);
        effort += Math.abs(power - lastPower);
        lastPower = power;
        if (time > duration * 0.65) {
            tailTime += dt;
            tailError += error * dt;
            tailSquare += error * error * dt;
        }
    }

    void sampleTolerance(double time, boolean withinTolerance) {
        lastSample = time;
        if (!withinTolerance) withinToleranceSince = Double.NaN;
        else if (!Double.isFinite(withinToleranceSince)) withinToleranceSince = time;
    }

    double settlingTime(double hold) {
        return lastSample - withinToleranceSince >= hold
                ? withinToleranceSince + hold : Double.POSITIVE_INFINITY;
    }

    double scoreWithSettlingTime(double hold) {
        return 1.65 * settlingTime(hold) / duration + 0.50 * score();
    }

    double overshoot() { return overshoot; }

    double score() {
        if (tailTime < duration * 0.12) return Double.POSITIVE_INFINITY;
        double mean = tailError / tailTime;
        double deviation = Math.sqrt(Math.max(0, tailSquare / tailTime - mean * mean));
        return errorArea / duration + 1.8 * Math.abs(mean) + 2 * deviation + 1.5 * overshoot + 0.07 * effort;
    }
}
