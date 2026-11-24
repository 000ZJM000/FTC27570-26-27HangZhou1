package org.firstinspires.ftc.teamcode.biobuzz;

final class AutoVolley {
    static final double FEED_SECONDS = 2.0;
    private static final double MAX_EXTRA_SECONDS = 2.0;
    private final double deadline;

    private boolean feeding;
    private double feedSeconds, previousUpdate;

    AutoVolley(double start, double routeWaitSeconds) {
        previousUpdate = start;
        deadline = start + Math.max(routeWaitSeconds, FEED_SECONDS) + MAX_EXTRA_SECONDS;
    }

    boolean requesting(double now) {
        if (feeding) feedSeconds += now - previousUpdate;
        previousUpdate = now;
        return now < deadline && !finished();
    }

    void recordOutput(boolean feeding) { this.feeding = feeding; }

    boolean finished() { return feedSeconds >= FEED_SECONDS; }
    double feedSeconds() { return feedSeconds; }
}
