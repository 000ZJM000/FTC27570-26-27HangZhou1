package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.ArrayList;
import java.util.List;

/** Shot vectors at tile centers and optional nearby points in a half of the field. */
public final class ShotVectorField {
    public static final int COLUMNS = 3, ROWS = 6;
    private static final int RECORD_SIZE = 3;
    private static final Corner[] CORNERS = Corner.values();
    private static final int LEGACY_REFINEMENTS_PER_CELL = 8;
    private static final int REFINE_RECORD_SIZE = 5;
    private final CalibrationStore store;
    private final String key, refinementKey;
    private final int firstColumn;
    // Each record is [unreachable, forward component, right component].
    private double[] records;
    // Positions keep saved shots tied to their actual pose when the reachability map changes.
    private double[] refinedRecords;
    private final double[] legacyRecords;
    private final Refinement[] refinements = new Refinement[ROWS * COLUMNS * CORNERS.length];
    private final Refinement[] legacyRefinements = new Refinement[ROWS * COLUMNS * LEGACY_REFINEMENTS_PER_CELL];

    public ShotVectorField(CalibrationStore store, boolean rightHalf) {
        this.store = store;
        key = rightHalf ? "shot_vector_field_right_v1" : "shot_vector_field_left_v1";
        String refinementPrefix = rightHalf ? "shot_vector_refinements_right" : "shot_vector_refinements_left";
        refinementKey = refinementPrefix + "_v2";
        firstColumn = rightHalf ? COLUMNS : 0;
        records = store.readExact(key, ROWS * COLUMNS * RECORD_SIZE);
        if (records == null) records = new double[ROWS * COLUMNS * RECORD_SIZE];
        refinedRecords = store.readExact(refinementKey, refinements.length * REFINE_RECORD_SIZE);
        if (refinedRecords == null) refinedRecords = new double[refinements.length * REFINE_RECORD_SIZE];
        legacyRecords = store.readExact(refinementPrefix + "_v1", legacyRefinements.length * REFINE_RECORD_SIZE);
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
        return save(column, row, shot, -1, -1);
    }

    public boolean save(int column, int row, Vector shot, double battery, double pitch) {
        int index = offset(column, row);
        if (records[index] == 1 || !valid(shot)) return false;
        double[] next = records.clone();
        next[index + 1] = shot.forward;
        next[index + 2] = shot.right;
        String id = pointId(row + 1, firstColumn + column + 1, '0');
        if (!store.saveShot(key, next, id, battery, pitch)) return false;
        records = next;
        generateRefinements();
        return true;
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

    List<Refinement> refinementSamples() {
        List<Refinement> result = refinements();
        for (Refinement point : legacyRefinements) {
            if (point != null) result.add(point);
        }
        return result;
    }

    /** Lists the saved values even when a point has since been marked unreachable or disabled. */
    public List<PointData> pointsForReview() {
        List<PointData> points = new ArrayList<>();
        for (int row = 0; row < ROWS; row++) {
            for (int column = 0; column < COLUMNS; column++) {
                int fieldColumn = firstColumn + column + 1;
                int index = offset(column, row);
                points.add(new PointData(row + 1, fieldColumn, "格心", '0', centerRightTiles(column),
                        centerForwardTiles(row), vector(records[index + 1], records[index + 2]),
                        records[index] == 1, false));

                int cell = row * COLUMNS + column;
                for (Corner corner : CORNERS) {
                    int slot = cell * CORNERS.length + corner.ordinal();
                    String label = corner.label + "角";
                    char code = (char) ('1' + corner.ordinal());
                    Refinement current = refinements[slot];
                    boolean savedAtCurrentPose = addSavedRefinement(points, row + 1, fieldColumn, label, code,
                            refinedRecords, slot, current);
                    if (current != null && !savedAtCurrentPose) {
                        points.add(new PointData(row + 1, fieldColumn, label, code, current.rightTiles,
                                current.forwardTiles, null, false, false));
                    }
                }
                if (legacyRecords == null) continue;
                for (int i = 0; i < LEGACY_REFINEMENTS_PER_CELL; i++) {
                    int slot = cell * LEGACY_REFINEMENTS_PER_CELL + i;
                    addSavedRefinement(points, row + 1, fieldColumn, "旧精测 " + (i + 1), (char) ('a' + i),
                            legacyRecords, slot, legacyRefinements[slot]);
                }
            }
        }
        return points;
    }

    private static boolean addSavedRefinement(List<PointData> points, int row, int column, String label, char code,
                                             double[] saved, int slot, Refinement current) {
        int index = slot * REFINE_RECORD_SIZE;
        boolean unreachable = saved[index] == 1;
        Vector shot = vector(saved[index + 3], saved[index + 4]);
        if (!unreachable && shot == null) return false;
        boolean currentPose = current != null && matches(current, saved);
        points.add(new PointData(row, column, label, code, saved[index + 1], saved[index + 2], shot,
                unreachable, !currentPose));
        return currentPose;
    }

    public boolean isUnreachable(Refinement point) {
        double[] saved = point.corner == null ? legacyRecords : refinedRecords;
        return matches(point, saved) && saved[point.slot * REFINE_RECORD_SIZE] == 1;
    }

    public Vector at(Refinement point) {
        double[] saved = point.corner == null ? legacyRecords : refinedRecords;
        int index = point.slot * REFINE_RECORD_SIZE;
        return matches(point, saved) && saved[index] == 0 ? vector(saved[index + 3], saved[index + 4]) : null;
    }

    public boolean save(Refinement point, Vector shot) {
        return save(point, shot, -1, -1);
    }

    public boolean save(Refinement point, Vector shot, double battery, double pitch) {
        return point.corner != null && refinements[point.slot] == point && !isUnreachable(point) && valid(shot)
                && writeRefinement(point, false, shot.forward, shot.right, new double[]{battery, pitch});
    }

    public boolean setUnreachable(Refinement point, boolean unreachable) {
        if (point.corner == null || refinements[point.slot] != point) return false;
        int index = point.slot * REFINE_RECORD_SIZE;
        boolean samePosition = matches(point, refinedRecords);
        return writeRefinement(point, unreachable, samePosition ? refinedRecords[index + 3] : 0,
                samePosition ? refinedRecords[index + 4] : 0, null);
    }

    private static boolean matches(Refinement point, double[] saved) {
        int index = point.slot * REFINE_RECORD_SIZE;
        return saved[index + 1] == point.rightTiles && saved[index + 2] == point.forwardTiles;
    }

    private boolean writeRefinement(Refinement point, boolean unreachable, double forward, double right, double[] metadata) {
        double[] next = refinedRecords.clone();
        int index = point.slot * REFINE_RECORD_SIZE;
        next[index] = unreachable ? 1 : 0;
        next[index + 1] = point.rightTiles;
        next[index + 2] = point.forwardTiles;
        next[index + 3] = forward;
        next[index + 4] = right;
        String id = pointId(point.row + 1, point.fieldColumn, (char) ('1' + point.corner.ordinal()));
        boolean saved = metadata == null ? store.save(refinementKey, next)
                : store.saveShot(refinementKey, next, id, metadata[0], metadata[1]);
        if (!saved) return false;
        refinedRecords = next;
        generateRefinements();
        return true;
    }

    private void generateRefinements() {
        boolean fitsTile = RobotConfig.ROBOT_WIDTH_MM <= RobotConfig.TILE_MM
                && RobotConfig.ROBOT_LENGTH_MM <= RobotConfig.TILE_MM;
        for (int row = 0; row < ROWS; row++) {
            for (int column = 0; column < COLUMNS; column++) {
                boolean measured = measured(column, row);
                int cell = row * COLUMNS + column;
                for (Corner corner : CORNERS) {
                    int slot = cell * CORNERS.length + corner.ordinal();
                    if (!measured || !fitsTile) refinements[slot] = null;
                    else if (refinements[slot] == null) refinements[slot] = cornerPoint(slot, column, row, corner);
                }
                if (legacyRecords == null) continue;
                for (int i = 0; i < LEGACY_REFINEMENTS_PER_CELL; i++) {
                    int slot = cell * LEGACY_REFINEMENTS_PER_CELL + i;
                    legacyRefinements[slot] = measured && !hasCornerRecord(cell) ? storedLegacyPoint(slot, column, row) : null;
                }
            }
        }
    }

    private boolean hasCornerRecord(int cell) {
        for (int i = 0; i < CORNERS.length; i++) {
            int slot = cell * CORNERS.length + i;
            Refinement point = refinements[slot];
            if (point != null && (isUnreachable(point) || at(point) != null)) return true;
        }
        return false;
    }

    static String pointId(int row, int column, char code) { return "" + row + column + code; }

    private Refinement cornerPoint(int slot, int column, int row, Corner corner) {
        double tile = RobotConfig.TILE_MM;
        double right = firstColumn + column + (corner.onRight
                ? 1 - RobotConfig.CENTER_FROM_RIGHT_EDGE_MM / tile : RobotConfig.CENTER_FROM_LEFT_EDGE_MM / tile);
        double forward = row + (corner.atFront
                ? 1 - RobotConfig.CENTER_FROM_FRONT_EDGE_MM / tile : RobotConfig.CENTER_FROM_REAR_EDGE_MM / tile);
        return new Refinement(slot, column, firstColumn + column + 1, row, right, forward, corner);
    }

    private Refinement storedLegacyPoint(int slot, int column, int row) {
        int index = slot * REFINE_RECORD_SIZE;
        if (legacyRecords[index] == 0 && legacyRecords[index + 3] == 0 && legacyRecords[index + 4] == 0) {
            return null;
        }
        double right = legacyRecords[index + 1], forward = legacyRecords[index + 2];
        int fieldColumn = firstColumn + column;
        if (right <= fieldColumn || right >= fieldColumn + 1 || forward <= row || forward >= row + 1
                || !fitsLegacyPose(right, forward)) return null;
        return new Refinement(slot, column, fieldColumn + 1, row, right, forward, null);
    }

    private boolean fitsLegacyPose(double right, double forward) {
        double clearanceMm = 20;
        double leftEdge = right - (RobotConfig.CENTER_FROM_LEFT_EDGE_MM + clearanceMm) / RobotConfig.TILE_MM;
        double rightEdge = right + (RobotConfig.CENTER_FROM_RIGHT_EDGE_MM + clearanceMm) / RobotConfig.TILE_MM;
        double rearEdge = forward - (RobotConfig.CENTER_FROM_REAR_EDGE_MM + clearanceMm) / RobotConfig.TILE_MM;
        double frontEdge = forward + (RobotConfig.CENTER_FROM_FRONT_EDGE_MM + clearanceMm) / RobotConfig.TILE_MM;
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

    /** Shooting permission is independent of the interpolation used for aiming. */
    boolean allowsShot(double right, double forward) {
        if (!Maths.finite(right, forward) || right < firstColumn || right >= firstColumn + COLUMNS
                || forward < 0 || forward >= ROWS) return false;
        int column = (int) right - firstColumn, row = (int) forward;
        return measured(column, row) && !nearUnreachableRefinement(column, row, right, forward);
    }

    private boolean nearUnreachableRefinement(int column, int row, double right, double forward) {
        boolean blocked = false;
        double nearest = squaredDistance(right, forward, centerRightTiles(column), centerForwardTiles(row));
        int cell = row * COLUMNS + column;
        for (int i = 0; i < CORNERS.length + LEGACY_REFINEMENTS_PER_CELL; i++) {
            Refinement point = i < CORNERS.length ? refinements[cell * CORNERS.length + i]
                    : legacyRefinements[cell * LEGACY_REFINEMENTS_PER_CELL + i - CORNERS.length];
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

    private static int offset(int column, int row) {
        if (column < 0 || column >= COLUMNS || row < 0 || row >= ROWS) {
            throw new IndexOutOfBoundsException("Shot field cell outside the selected half");
        }
        return (row * COLUMNS + column) * RECORD_SIZE;
    }

    public enum Corner {
        REAR_LEFT("左后", false, false),
        REAR_RIGHT("右后", true, false),
        FRONT_RIGHT("右前", true, true),
        FRONT_LEFT("左前", false, true);

        public final String label;
        private final boolean onRight, atFront;

        Corner(String label, boolean onRight, boolean atFront) {
            this.label = label;
            this.onRight = onRight;
            this.atFront = atFront;
        }
    }

    public static final class Refinement {
        public final int column, fieldColumn, row;
        public final double rightTiles, forwardTiles;
        public final Corner corner;
        private final int slot;

        private Refinement(int slot, int column, int fieldColumn, int row, double right, double forward, Corner corner) {
            this.slot = slot;
            this.column = column;
            this.fieldColumn = fieldColumn;
            this.row = row;
            this.corner = corner;
            rightTiles = right;
            forwardTiles = forward;
        }
    }

    public static final class PointData {
        public final int row, column;
        public final String label;
        public final char code;
        public final double rightTiles, forwardTiles;
        public final Vector shot;
        public final boolean unreachable, inactive;

        private PointData(int row, int column, String label, char code, double right, double forward, Vector shot,
                          boolean unreachable, boolean inactive) {
            this.row = row;
            this.column = column;
            this.label = label;
            this.code = code;
            this.rightTiles = right;
            this.forwardTiles = forward;
            this.shot = shot;
            this.unreachable = unreachable;
            this.inactive = inactive;
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
