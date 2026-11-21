package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** A smooth field of shot bearings and RPM derived from 06. Coordinates are in tiles. */
final class CalibratedShotMap {
    private static final double FIELD = FieldTargets.FIELD_TILES;
    private static final double MIDLINE = FIELD / 2;
    private final ShotVectorField source;
    private final boolean fromRight;
    private final Point[] points;
    private final double mirrorAxis;

    CalibratedShotMap(CalibrationStore store) {
        ShotVectorField left = new ShotVectorField(store, false);
        ShotVectorField right = new ShotVectorField(store, true);
        fromRight = centerCount(right) > centerCount(left);
        source = fromRight ? right : left;
        List<Point> recorded = new ArrayList<>();
        for (int row = 0; row < ShotVectorField.ROWS; row++) {
            for (int column = 0; column < ShotVectorField.COLUMNS; column++) {
                add(recorded, source.centerRightTiles(column), source.centerForwardTiles(row), source.at(column, row));
            }
        }
        for (ShotVectorField.Refinement point : source.refinementSamples()) {
            add(recorded, point.rightTiles, point.forwardTiles, source.at(point));
        }
        points = new Point[2 * recorded.size()];
        if (recorded.isEmpty()) {
            mirrorAxis = 0;
            return;
        }

        unwrap(recorded);
        double mean = 0;
        for (Point point : recorded) mean += point.angle;
        // The continuous mirror can cross the midline facing either forward or backward.
        mirrorAxis = Math.rint(mean / (recorded.size() * Math.PI)) * Math.PI;
        for (int i = 0; i < recorded.size(); i++) {
            Point point = recorded.get(i);
            point.fitSlope(recorded);
        }
        for (int i = 0; i < recorded.size(); i++) {
            Point point = recorded.get(i);
            point.angle -= mirrorAxis;
            points[2 * i] = point;
            points[2 * i + 1] = point.mirrored();
        }
    }

    String description() {
        if (points.length == 0) return "等待 06 格心数据";
        return fromRight ? "右半场实测，镜像到左半场" : "左半场实测，镜像到右半场";
    }

    boolean allowsShot(double right, double forward) {
        if (!(right >= 0 && right < FIELD)) return false;
        double sourceRight = (right >= MIDLINE) == fromRight ? right : FIELD - right;
        double upperEdge = fromRight ? FIELD : MIDLINE;
        return source.allowsShot(Math.min(sourceRight, Math.nextDown(upperEdge)), forward);
    }

    ShotVectorField.Vector sample(double right, double forward) {
        if (points.length == 0 || !Maths.finite(right, forward)) return null;
        double angle = 0, rpm = 0, total = 0;
        // Modified Shepard interpolation. Every point participates, so tile and half boundaries
        // cannot change the stencil. The local slopes also carry through each measured point.
        for (Point point : points) {
            double dx = right - point.right, dy = forward - point.forward;
            double distanceSquared = dx * dx + dy * dy;
            if (distanceSquared == 0) return ShotVectorField.Vector.fromShot(mirrorAxis + point.angle, point.rpm);
            double weight = 1 / (distanceSquared * distanceSquared);
            double localAngle = point.angle + (point.slopeX * dx + point.slopeY * dy) / (1 + distanceSquared);
            angle += weight * localAngle;
            rpm += weight * point.rpm;
            total += weight;
        }
        return ShotVectorField.Vector.fromShot(mirrorAxis + angle / total, rpm / total);
    }

    private static int centerCount(ShotVectorField field) {
        int count = 0;
        for (int row = 0; row < ShotVectorField.ROWS; row++) {
            for (int column = 0; column < ShotVectorField.COLUMNS; column++) {
                if (field.at(column, row) != null) count++;
            }
        }
        return count;
    }

    private static void add(List<Point> points, double right, double forward, ShotVectorField.Vector shot) {
        if (shot != null) points.add(new Point(right, forward, shot.bearing(), shot.rpm()));
    }

    private static void unwrap(List<Point> points) {
        int count = points.size();
        boolean[] placed = new boolean[count];
        double[] distance = new double[count];
        int[] neighbor = new int[count];
        Arrays.fill(distance, Double.POSITIVE_INFINITY);
        int next = 0;
        for (int i = 0; i < count; i++) {
            Point current = points.get(next);
            if (i > 0) {
                double reference = points.get(neighbor[next]).angle;
                current.angle = reference + Maths.wrap(current.angle - reference);
            }
            placed[next] = true;
            int closest = -1;
            for (int j = 0; j < count; j++) {
                if (placed[j]) continue;
                double dx = current.right - points.get(j).right, dy = current.forward - points.get(j).forward;
                double squared = dx * dx + dy * dy;
                if (squared < distance[j]) {
                    distance[j] = squared;
                    neighbor[j] = next;
                }
                if (closest < 0 || distance[j] < distance[closest]) closest = j;
            }
            next = closest;
        }
    }

    private static final class Point {
        final double right, forward, rpm;
        double angle, slopeX, slopeY;

        Point(double right, double forward, double angle, double rpm) {
            this.right = right;
            this.forward = forward;
            this.angle = angle;
            this.rpm = rpm;
        }

        void fitSlope(List<Point> points) {
            // Small ridge keeps the unmeasured lateral slope bounded with a single sampled column.
            double xx = 0.10, xy = 0, yy = 0.10, xAngle = 0, yAngle = 0;
            for (Point point : points) {
                if (point == this) continue;
                double dx = point.right - right, dy = point.forward - forward;
                double distance = dx * dx + dy * dy;
                if (distance > 1.5 * 1.5) continue;
                double weight = 1 / (0.06 + distance);
                double da = point.angle - angle;
                xx += weight * dx * dx;
                xy += weight * dx * dy;
                yy += weight * dy * dy;
                xAngle += weight * dx * da;
                yAngle += weight * dy * da;
            }
            double determinant = xx * yy - xy * xy;
            slopeX = (yy * xAngle - xy * yAngle) / determinant;
            slopeY = (xx * yAngle - xy * xAngle) / determinant;
            double scale = Math.max(1, Math.hypot(slopeX, slopeY) / Math.PI);
            slopeX /= scale;
            slopeY /= scale;
        }

        Point mirrored() {
            Point point = new Point(FIELD - right, forward, -angle, rpm);
            point.slopeX = slopeX;
            point.slopeY = -slopeY;
            return point;
        }
    }
}
