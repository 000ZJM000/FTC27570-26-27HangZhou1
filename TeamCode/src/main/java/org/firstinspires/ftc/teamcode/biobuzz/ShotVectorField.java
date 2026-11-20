package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.ArrayList;
import java.util.List;

/** Shot vectors at tile centers and optional nearby points in a half of the field. */
public final class ShotVectorField {
    public static final int COLUMNS = 3, ROWS = 6;
    private static final int RECORD_SIZE = 3;
    private static final int REFINEMENTS_PER_CELL = 8;
    private static final int REFINE_RECORD_SIZE = 5;
    private static final double REFINE_STEP_TILES = 0.25;
    private static final double CLEARANCE_MM = 20;
    private static final double MIN_SPACING_TILES = 20 / RobotConfig.TILE_MM;
    private static final double SAMPLE_RADIUS_SQUARED = 1.0;
    private final CalibrationStore store;
    private final String key, refinementKey;
    private final int firstColumn;
    // Each record is [unreachable, forward component, right component].
    private double[] records;
    // Positions keep saved shots tied to their actual pose when the reachability map changes.
    private double[] refinedRecords;
    private final Refinement[] refinements = new Refinement[ROWS * COLUMNS * REFINEMENTS_PER_CELL];

    public ShotVectorField(CalibrationStore store, boolean rightHalf) {
        this.store = store;
        key = rightHalf ? "shot_vector_field_right_v1" : "shot_vector_field_left_v1";
        refinementKey = rightHalf ? "shot_vector_refinements_right_v1" : "shot_vector_refinements_left_v1";
        firstColumn = rightHalf ? COLUMNS : 0;
        records = store.readExact(key, ROWS * COLUMNS * RECORD_SIZE);
        if (records == null) records = new double[ROWS * COLUMNS * RECORD_SIZE];
        refinedRecords = store.readExact(refinementKey, refinements.length * REFINE_RECORD_SIZE);
        if (refinedRecords == null) refinedRecords = new double[refinements.length * REFINE_RECORD_SIZE];
        generateRefinements();
    }

    public double centerRightTiles(int column) { return firstColumn + column + 0.5; }
    public double centerForwardTiles(int row) { return row + 0.5; }

    public boolean isUnreachable(int column, int row) {
        return records[offset(column, row)] == 1;
    }

    /** Returns null for an unmeasured or unreachable tile. */
    public Vector at(int column, int row) {
        int index = offset(column, row);
        return records[index] == 1 ? null : vector(records[index + 1], records[index + 2]);
    }

    public boolean save(int column, int row, Vector shot) {
        int index = offset(column, row);
        if (records[index] == 1 || !valid(shot)) return false;
        double[] next = records.clone();
        next[index + 1] = shot.forward;
        next[index + 2] = shot.right;
        return write(next);
    }

    public boolean setUnreachable(int column, int row, boolean unreachable) {
        double[] next = records.clone();
        next[offset(column, row)] = unreachable ? 1 : 0;
        return write(next);
    }

    private boolean write(double[] next) {
        if (!store.save(key, next)) return false;
        records = next;
        generateRefinements();
        return true;
    }

    public List<Refinement> refinements() {
        List<Refinement> result = new ArrayList<>();
        for (Refinement point : refinements) {
            if (point != null) result.add(point);
        }
        return result;
    }

    public boolean isUnreachable(Refinement point) {
        return matches(point) && refinedRecords[point.slot * REFINE_RECORD_SIZE] == 1;
    }

    public Vector at(Refinement point) {
        int index = point.slot * REFINE_RECORD_SIZE;
        return matches(point) && refinedRecords[index] == 0
                ? vector(refinedRecords[index + 3], refinedRecords[index + 4]) : null;
    }

    public boolean save(Refinement point, Vector shot) {
        return refinements[point.slot] == point && !isUnreachable(point) && valid(shot)
                && writeRefinement(point, false, shot.forward, shot.right);
    }

    public boolean setUnreachable(Refinement point, boolean unreachable) {
        if (refinements[point.slot] != point) return false;
        int index = point.slot * REFINE_RECORD_SIZE;
        boolean samePosition = matches(point);
        return writeRefinement(point, unreachable, samePosition ? refinedRecords[index + 3] : 0,
                samePosition ? refinedRecords[index + 4] : 0);
    }

    private boolean matches(Refinement point) {
        int index = point.slot * REFINE_RECORD_SIZE;
        return refinedRecords[index + 1] == point.rightTiles && refinedRecords[index + 2] == point.forwardTiles;
    }

    private boolean writeRefinement(Refinement point, boolean unreachable, double forward, double right) {
        double[] next = refinedRecords.clone();
        int index = point.slot * REFINE_RECORD_SIZE;
        next[index] = unreachable ? 1 : 0;
        next[index + 1] = point.rightTiles;
        next[index + 2] = point.forwardTiles;
        next[index + 3] = forward;
        next[index + 4] = right;
        if (!store.save(refinementKey, next)) return false;
        refinedRecords = next;
        return true;
    }

    private void generateRefinements() {
        int slot = 0;
        for (int row = 0; row < ROWS; row++) {
            for (int column = 0; column < COLUMNS; column++) {
                boolean measured = at(column, row) != null;
                for (int forward = -1; forward <= 1; forward++) {
                    for (int right = -1; right <= 1; right++) {
                        if (forward == 0 && right == 0) continue;
                        refinements[slot] = measured ? refine(slot, column, row, right, forward) : null;
                        slot++;
                    }
                }
            }
        }
    }

    private Refinement refine(int slot, int column, int row, int right, int forward) {
        double x = centerRightTiles(column), y = centerForwardTiles(row);
        double dx = right * REFINE_STEP_TILES, dy = forward * REFINE_STEP_TILES;
        double fraction = 1;
        if (!fits(x + dx, y + dy)) {
            // The center fits. Shorten the offset along this ray to the clearance boundary.
            double low = 0, high = 1;
            for (int i = 0; i < 14; i++) {
                double middle = (low + high) / 2;
                if (fits(x + middle * dx, y + middle * dy)) low = middle;
                else high = middle;
            }
            fraction = low;
        }
        if (fraction * Math.hypot(dx, dy) < MIN_SPACING_TILES) return null;
        return new Refinement(slot, column, row, x + fraction * dx, y + fraction * dy);
    }

    private boolean fits(double right, double forward) {
        double leftEdge = right - (RobotConfig.CENTER_FROM_LEFT_EDGE_MM + CLEARANCE_MM) / RobotConfig.TILE_MM;
        double rightEdge = right + (RobotConfig.CENTER_FROM_RIGHT_EDGE_MM + CLEARANCE_MM) / RobotConfig.TILE_MM;
        double rearEdge = forward - (RobotConfig.CENTER_FROM_REAR_EDGE_MM + CLEARANCE_MM) / RobotConfig.TILE_MM;
        double frontEdge = forward + (RobotConfig.CENTER_FROM_FRONT_EDGE_MM + CLEARANCE_MM) / RobotConfig.TILE_MM;
        if (leftEdge < firstColumn || rightEdge > firstColumn + COLUMNS
                || rearEdge < 0 || frontEdge > FieldTargets.FIELD_TILES) return false;
        for (int r = 0; r < ROWS; r++) {
            if (rearEdge >= r + 1 || frontEdge <= r) continue;
            for (int c = 0; c < COLUMNS; c++) {
                int edge = firstColumn + c;
                if (leftEdge < edge + 1 && rightEdge > edge && isUnreachable(c, r)) return false;
            }
        }
        return true;
    }

    /** Looks up a shot at a field position expressed in tiles. */
    public Vector sample(double right, double forward) {
        if (!Maths.finite(right, forward) || right < firstColumn || right >= firstColumn + COLUMNS
                || forward < 0 || forward >= ROWS) return null;
        int column = (int) right - firstColumn, row = (int) forward;
        if (!measured(column, row) || nearUnreachableRefinement(column, row, right, forward)) return null;

        Blend blend = new Blend();
        for (int r = Math.max(0, row - 1); r <= Math.min(ROWS - 1, row + 1); r++) {
            for (int c = Math.max(0, column - 1); c <= Math.min(COLUMNS - 1, column + 1); c++) {
                if (r != row && c != column && (!measured(column, r) || !measured(c, row))) continue;
                Vector center = at(c, r);
                if (center == null) continue;
                if (blend.add(center, squaredDistance(right, forward, centerRightTiles(c), centerForwardTiles(r)))) {
                    return center;
                }
                int first = (r * COLUMNS + c) * REFINEMENTS_PER_CELL;
                for (int i = first; i < first + REFINEMENTS_PER_CELL; i++) {
                    Refinement point = refinements[i];
                    if (point == null) continue;
                    Vector shot = at(point);
                    if (shot != null && blend.add(shot,
                            squaredDistance(right, forward, point.rightTiles, point.forwardTiles))) return shot;
                }
            }
        }
        return blend.result();
    }

    private boolean nearUnreachableRefinement(int column, int row, double right, double forward) {
        boolean blocked = false;
        double nearest = squaredDistance(right, forward, centerRightTiles(column), centerForwardTiles(row));
        int first = (row * COLUMNS + column) * REFINEMENTS_PER_CELL;
        for (int i = first; i < first + REFINEMENTS_PER_CELL; i++) {
            Refinement point = refinements[i];
            if (point == null) continue;
            boolean unreachable = isUnreachable(point);
            if (!unreachable && at(point) == null) continue;
            double distance = squaredDistance(right, forward, point.rightTiles, point.forwardTiles);
            if (distance < nearest || (distance == nearest && unreachable)) {
                nearest = distance;
                blocked = unreachable;
            }
        }
        return blocked;
    }

    private boolean measured(int column, int row) {
        int index = offset(column, row);
        return records[index] == 0 && (records[index + 1] != 0 || records[index + 2] != 0);
    }

    private static double squaredDistance(double right, double forward, double x, double y) {
        double dx = x - right, dy = y - forward;
        return dx * dx + dy * dy;
    }

    private static boolean valid(Vector shot) {
        return shot != null && Double.isFinite(shot.rpm()) && shot.rpm() > 0;
    }

    private static Vector vector(double forward, double right) {
        return forward == 0 && right == 0 ? null : new Vector(forward, right);
    }

    private static final class Blend {
        private double forward, right, rpm, weightSum;

        boolean add(Vector shot, double distanceSquared) {
            if (distanceSquared < 1e-12) return true;
            if (distanceSquared >= SAMPLE_RADIUS_SQUARED) return false;
            double taper = 1 / distanceSquared - 1 / SAMPLE_RADIUS_SQUARED;
            double weight = taper * taper;
            double speed = shot.rpm();
            forward += weight * shot.forward / speed;
            right += weight * shot.right / speed;
            rpm += weight * speed;
            weightSum += weight;
            return false;
        }

        Vector result() {
            if (weightSum == 0 || Math.hypot(forward, right) < 0.05 * weightSum) return null;
            return Vector.fromShot(Math.atan2(-right, forward), rpm / weightSum);
        }
    }

    private static int offset(int column, int row) {
        if (column < 0 || column >= COLUMNS || row < 0 || row >= ROWS) {
            throw new IndexOutOfBoundsException("Shot field cell outside the selected half");
        }
        return (row * COLUMNS + column) * RECORD_SIZE;
    }

    public static final class Refinement {
        public final int column, row;
        public final double rightTiles, forwardTiles;
        private final int slot;

        private Refinement(int slot, int column, int row, double right, double forward) {
            this.slot = slot;
            this.column = column;
            this.row = row;
            rightTiles = right;
            forwardTiles = forward;
        }
    }

    public static final class Vector {
        public final double forward, right;

        private Vector(double forward, double right) {
            this.forward = forward;
            this.right = right;
        }

        /** Bearing is zero straight forward and positive to the left, as in the main OpMode. */
        public static Vector fromShot(double bearing, double rpm) {
            if (!Double.isFinite(bearing) || !Double.isFinite(rpm) || rpm <= 0) {
                throw new IllegalArgumentException("A shot needs a finite bearing and positive RPM");
            }
            return new Vector(rpm * Math.cos(bearing), -rpm * Math.sin(bearing));
        }

        public double bearing() { return Maths.wrap(Math.atan2(-right, forward)); }
        public double rpm() { return Math.hypot(forward, right); }
    }
}
