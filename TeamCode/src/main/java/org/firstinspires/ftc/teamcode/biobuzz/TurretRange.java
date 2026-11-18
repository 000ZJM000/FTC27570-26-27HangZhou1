package org.firstinspires.ftc.teamcode.biobuzz;

public final class TurretRange {
    private final double low, high;
    private int direction;
    private boolean rewindTargetMoves;
    private double previousTarget = Double.NaN;
    public boolean rewinding, reachable, predicted;

    public TurretRange(double span) {
        low = RobotConfig.TURRET_MARGIN;
        high = Math.min(span, Maths.TURN) - RobotConfig.TURRET_MARGIN;
    }

    public void reset() {
        finishRewind();
        reachable = false;
        previousTarget = Double.NaN;
    }

    public void finishRewind() {
        direction = 0;
        rewinding = predicted = false;
    }

    boolean rewindTargetMoves() { return rewindTargetMoves; }

    public double target(double bearing, double current, double relativeRate, double lookahead) {
        double equivalent = equivalent(bearing);
        reachable = equivalent <= high;
        if (!rewinding) {
            double forecast = equivalent(bearing + relativeRate * lookahead);
            double nearCurrent = current + Maths.wrap(bearing - current);
            double edge = relativeRate > 0 ? high : low;
            boolean atEdge = Math.abs(current - edge) <= Math.toRadians(26)
                    && Math.abs(previousTarget - edge) <= Math.toRadians(26);
            boolean towardEdge = (nearCurrent - edge) * Math.signum(relativeRate) >= -Math.toRadians(12);
            boolean crosses = Math.abs(relativeRate) > 0.20 && atEdge && towardEdge && forecast <= high
                    && (edge - forecast) * Math.signum(relativeRate) > Math.PI;
            if (crosses) start(-Math.signum(relativeRate), true);
            else if (reachable && Math.abs(equivalent - previousTarget) > Math.PI
                    && Math.abs(equivalent - current) > Math.PI) {
                start(Math.signum(equivalent - current), false);
            }
            // An initial acquisition or a newly selected field goal takes the normal PID path.
            if (!rewinding) {
                previousTarget = reachable ? equivalent : Maths.clip(nearCurrent, low, high);
                return previousTarget;
            }
        }
        double destination = direction > 0 ? high : low;
        double branch = destination + Maths.wrap(bearing - destination);
        if (reachable && (equivalent - destination) * direction > -Math.PI) branch = equivalent;
        rewindTargetMoves = branch > low && branch < high;
        previousTarget = Maths.clip(branch, low, high);
        return previousTarget;
    }

    private void start(double direction, boolean predicted) {
        this.direction = (int) direction;
        this.predicted = predicted;
        rewinding = true;
    }

    private double equivalent(double bearing) {
        double offset = (bearing - low) % Maths.TURN;
        return low + (offset < 0 ? offset + Maths.TURN : offset);
    }
}
