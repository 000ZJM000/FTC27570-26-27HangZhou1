package org.firstinspires.ftc.teamcode.biobuzz.shooting;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.biobuzz.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Parametric half-field shot map (local Hub file -> APK bundled fallback -> 06 records fit) + JSON export. */
public final class ShotMap {
    private static final double FIELD = Aiming.FieldTargets.FIELD_TILES;
    private static final double MIDLINE = FIELD / 2;
    public static final String FILE = "shot_model.json";
    public static final String RECORDS_FILE = "shot_records.json";

    private enum Source { LOCAL_FILE, BUNDLED_ASSET, FITTED_FROM_06, NONE }

    private final Model model;
    private final Source source;

    public ShotMap(CalibrationStore store) {
        Model loaded = tryRead(store.readLocalJson(FILE));
        if (loaded != null) {
            model = loaded;
            source = Source.LOCAL_FILE;
            return;
        }
        loaded = tryRead(store.readBundledAssetJson(FILE));
        if (loaded != null) {
            model = loaded;
            source = Source.BUNDLED_ASSET;
            return;
        }
        model = fit(recordPoints(store));
        source = model != null ? Source.FITTED_FROM_06 : Source.NONE;
    }

    private static Model tryRead(JSONObject document) {
        if (document == null) return null;
        try {
            return readModel(document);
        } catch (JSONException invalid) {
            RobotLog.ww("BioBuzzShot", "shot_model.json: %s", invalid.toString());
            return null;
        }
    }

    public String description() {
        switch (source) {
            case LOCAL_FILE: return "已加载本机 shot_model.json";
            case BUNDLED_ASSET: return "已加载 APK 内置 shot_model.json";
            case FITTED_FROM_06: return "从 06 实测拟合";
            default: return "等待 06 实测（回退几何自瞄）";
        }
    }

    public ShotVectorField.Vector preview(double chassisRight, double chassisForward) {
        return sample(chassisRight + RobotConfig.TURRET_RIGHT_MM / RobotConfig.TILE_MM,
                chassisForward + RobotConfig.TURRET_FORWARD_MM / RobotConfig.TILE_MM, chassisRight >= MIDLINE);
    }

    public boolean available() { return model != null; }
    public double goalRight(boolean rightHalf) { return rightHalf ? FIELD - model.goal[0] : model.goal[0]; }
    public double goalForward() { return model.goal[1]; }

    public ShotVectorField.Vector sample(double right, double forward, boolean rightHalf) {
        if (model == null || !Maths.finite(right, forward)) return null;
        double local = rightHalf ? FIELD - right : right;
        double deg = bearing(local, forward, model.goal) + model.angular(local, forward);
        double angle = Math.toRadians((rightHalf ? -deg : deg) + model.angleTrim);
        double radius = distance(local, forward, model.goal);
        double rpm = model.speed(radius) + model.rpmTrim;
        return Maths.finite(angle, rpm) && rpm > 0 ? ShotVectorField.Vector.fromShot(angle, Math.min(20_000, rpm)) : null;
    }

    public static double lineRate(double right, double forward, double rightRate, double forwardRate,
                                  double goalRight, double goalForward) {
        double x = right - goalRight, y = goalForward - forward;
        double rangeSquared = x * x + y * y;
        return rangeSquared > 1e-4 ? (y * rightRate + x * forwardRate) / rangeSquared : 0;
    }

    public static List<ShotVectorField.PointData> recordPoints(CalibrationStore store) {
        List<ShotVectorField.PointData> points = new ShotVectorField(store, false).pointsForReview();
        points.addAll(new ShotVectorField(store, true).pointsForReview());
        return points;
    }

    public static boolean exportRecords(CalibrationStore store) {
        try {
            JSONObject geometry = new JSONObject()
                    .put("tileMm", RobotConfig.TILE_MM)
                    .put("turretForwardMm", RobotConfig.TURRET_FORWARD_MM)
                    .put("turretRightMm", RobotConfig.TURRET_RIGHT_MM)
                    .put("leftEdgeMm", RobotConfig.CENTER_FROM_LEFT_EDGE_MM)
                    .put("rightEdgeMm", RobotConfig.CENTER_FROM_RIGHT_EDGE_MM)
                    .put("rearEdgeMm", RobotConfig.CENTER_FROM_REAR_EDGE_MM)
                    .put("frontEdgeMm", RobotConfig.CENTER_FROM_FRONT_EDGE_MM)
                    .put("pitchUnits", "ST servo position [0,1]");
            JSONArray exported = new JSONArray();
            for (ShotVectorField.PointData point : recordPoints(store)) {
                if (point.shot == null && !point.unreachable) continue;
                String id = ShotVectorField.pointId(point.row, point.column, point.code);
                double[] metadata = store.readExact("shot_meta_" + id, 2);
                exported.put(new JSONObject().put("id", id).put("x", point.rightTiles).put("y", point.forwardTiles)
                        .put("rightMm", point.rightTiles * RobotConfig.TILE_MM)
                        .put("forwardMm", point.forwardTiles * RobotConfig.TILE_MM)
                        .put("angle", point.shot == null ? JSONObject.NULL : Math.toDegrees(point.shot.bearing()))
                        .put("rpm", point.shot == null ? JSONObject.NULL : point.shot.rpm())
                        .put("blocked", point.unreachable).put("inactive", point.inactive).put("weight", 1)
                        .put("batteryVoltage", metadata == null || metadata[0] < 0 ? JSONObject.NULL : metadata[0])
                        .put("pitch", metadata == null || metadata[1] < 0 ? JSONObject.NULL : metadata[1]));
            }
            return store.writeJson(RECORDS_FILE, new JSONObject().put("version", 2).put("kind", "biobuzz-shot-records")
                    .put("geometry", geometry).put("points", exported));
        } catch (JSONException malformed) {
            RobotLog.ww("BioBuzzShot", "export: %s", malformed.toString());
            return false;
        }
    }

    private static double bearing(double x, double y, double[] goal) {
        return Math.toDegrees(Math.atan2(x - goal[0], goal[1] - y));
    }

    private static double wrapDegrees(double degrees) { return Math.toDegrees(Maths.wrap(Math.toRadians(degrees))); }
    private static double distance(double x, double y, double[] goal) { return Math.hypot(x - goal[0], y - goal[1]); }

    private static Model fit(List<ShotVectorField.PointData> measured) {
        List<double[]> points = new ArrayList<>();
        double tile = RobotConfig.TILE_MM;
        for (ShotVectorField.PointData point : measured) {
            if (point.shot == null || point.unreachable || point.inactive) continue;
            double x = point.rightTiles + RobotConfig.TURRET_RIGHT_MM / tile;
            double y = point.forwardTiles + RobotConfig.TURRET_FORWARD_MM / tile;
            double deg = Math.toDegrees(point.shot.bearing());
            boolean right = point.rightTiles >= MIDLINE;
            points.add(new double[]{right ? FIELD - x : x, y, right ? -deg : deg, point.shot.rpm()});
        }
        if (points.isEmpty()) return null;
        double[] goal = fitGoal(points);
        double[] nx = normalization(points, 0), ny = normalization(points, 1);
        double[][] radii = new double[points.size()][1];
        for (int i = 0; i < points.size(); i++) {
            double[] point = points.get(i);
            radii[i][0] = distance(point[0], point[1], goal);
        }
        double[] nr = normalization(Arrays.asList(radii), 0);
        Maths.LeastSquares angular = new Maths.LeastSquares(new double[]{1e-9, 0.18, 0.18}, null);
        Maths.LeastSquares rpm = new Maths.LeastSquares(new double[]{1e-9, 0.18, 0.72}, null);
        for (int i = 0; i < points.size(); i++) {
            double[] p = points.get(i);
            angular.add(new double[]{1, scale(p[0], nx), scale(p[1], ny)},
                    wrapDegrees(p[2] - bearing(p[0], p[1], goal)), 1);
            double r = scale(radii[i][0], nr);
            rpm.add(new double[]{1, r, r * r}, p[3], 1);
        }
        return new Model(goal, new int[][]{{0, 0}, {1, 0}, {0, 1}}, nx, ny,
                angular.solve(), nr, rpm.solve(), 0, 0);
    }

    private static double[] normalization(List<double[]> points, int column) {
        double mean = 0, square = 0;
        for (double[] point : points) {
            mean += point[column];
            square += point[column] * point[column];
        }
        mean /= points.size();
        return new double[]{mean, Math.sqrt(Math.max(0.0625, square / points.size() - mean * mean))};
    }

    private static double scale(double x, double[] normalization) {
        return Maths.clip((x - normalization[0]) / normalization[1], -3, 3);
    }

    private static double[] fitGoal(List<double[]> points) {
        Maths.LeastSquares seed = new Maths.LeastSquares(new double[]{0.018, 0.018}, new double[]{2, 2.5});
        for (double[] p : points) {
            double angle = Math.toRadians(p[2]);
            double c = Math.cos(angle), s = Math.sin(angle);
            seed.add(new double[]{c, s}, c * p[0] + s * p[1], 1);
        }
        double[] goal = seed.solve();
        goal[0] = Maths.clip(goal[0], 0, FIELD);
        goal[1] = Maths.clip(goal[1], 0, FIELD);
        for (int step = 0; step < 16; step++) {
            Maths.LeastSquares correction = new Maths.LeastSquares(new double[]{0.025, 0.025}, null);
            for (double[] p : points) {
                double dx = goal[0] - p[0], dy = goal[1] - p[1];
                double squared = Math.max(dx * dx + dy * dy, 0.01);
                correction.add(new double[]{-dy / squared, dx / squared},
                        Math.toRadians(wrapDegrees(p[2] - bearing(p[0], p[1], goal))), 1);
            }
            double[] delta = correction.solve();
            if (delta == null || Math.hypot(delta[0], delta[1]) < 1e-7) break;
            double loss = loss(points, goal);
            double[] next = goal;
            for (double fraction = 1; fraction >= 0.03125; fraction /= 2) {
                double[] trial = {Maths.clip(goal[0] + fraction * delta[0], 0, FIELD),
                        Maths.clip(goal[1] + fraction * delta[1], 0, FIELD)};
                if (loss(points, trial) < loss) {
                    next = trial;
                    break;
                }
            }
            if (next == goal) break;
            goal = next;
        }
        return goal;
    }

    private static double loss(List<double[]> points, double[] goal) {
        double sum = 0;
        for (double[] p : points) {
            double e = wrapDegrees(p[2] - bearing(p[0], p[1], goal));
            sum += e * e;
        }
        return sum;
    }

    private static Model readModel(JSONObject document) throws JSONException {
        if (document.optInt("version") != 2 || !"biobuzz-shot-model".equals(document.optString("kind"))) {
            throw new JSONException("unsupported format");
        }
        JSONObject model = document.getJSONObject("model");
        double[] goal = vector(model.getJSONArray("goal"), 2, 2);
        if (goal[0] < 0 || goal[0] > FIELD || goal[1] < 0 || goal[1] > FIELD) throw new JSONException("goal outside field");
        JSONObject angular = model.getJSONObject("angular"), speed = model.getJSONObject("speed");
        double[] nx = normalization(angular, "x"), ny = normalization(angular, "y"), nr = normalization(speed, "radius");
        JSONArray basis = angular.getJSONArray("terms");
        double[] coefficients = vector(angular.getJSONArray("coefficients"), basis.length(), basis.length());
        if (basis.length() > 10) throw new JSONException("angular order");
        int[][] terms = new int[basis.length()][2];
        for (int i = 0; i < terms.length; i++) {
            double[] term = vector(basis.getJSONArray(i), 2, 2);
            if (term[0] < 0 || term[1] < 0 || term[0] + term[1] > 3
                    || term[0] != (int) term[0] || term[1] != (int) term[1]) throw new JSONException("angular term");
            terms[i][0] = (int) term[0];
            terms[i][1] = (int) term[1];
        }
        double angleTrim = model.getDouble("angleTrim"), rpmTrim = model.getDouble("rpmTrim");
        if (!Maths.finite(angleTrim, rpmTrim) || Math.abs(angleTrim) > 45 || Math.abs(rpmTrim) > 1000) {
            throw new JSONException("trim outside range");
        }
        return new Model(goal, terms, nx, ny, coefficients, nr,
                vector(speed.getJSONArray("coefficients"), 1, 4), angleTrim, rpmTrim);
    }

    private static double[] normalization(JSONObject model, String key) throws JSONException {
        double[] n = vector(model.getJSONArray(key), 2, 2);
        if (!(n[1] >= 0.25)) throw new JSONException("invalid normalization");
        return n;
    }

    private static double[] vector(JSONArray source, int min, int max) throws JSONException {
        if (source.length() < min || source.length() > max) throw new JSONException("invalid vector length");
        double[] values = new double[source.length()];
        for (int i = 0; i < values.length; i++) values[i] = source.getDouble(i);
        if (!Maths.finite(values)) throw new JSONException("nonfinite vector");
        return values;
    }

    private static final class Model {
        final double[] goal, nx, ny, coefficients, nr, rpm;
        final int[][] terms;
        final double angleTrim, rpmTrim;

        Model(double[] goal, int[][] terms, double[] nx, double[] ny, double[] coefficients,
              double[] nr, double[] rpm, double angleTrim, double rpmTrim) {
            this.goal = goal;
            this.nx = nx;
            this.ny = ny;
            this.coefficients = coefficients;
            this.nr = nr;
            this.rpm = rpm;
            this.terms = terms;
            this.angleTrim = angleTrim;
            this.rpmTrim = rpmTrim;
        }

        double angular(double x, double y) {
            double u = scale(x, nx), v = scale(y, ny);
            double result = 0;
            for (int i = 0; i < terms.length; i++) {
                result += coefficients[i] * Math.pow(u, terms[i][0]) * Math.pow(v, terms[i][1]);
            }
            return result;
        }

        double speed(double radius) {
            double r = scale(radius, nr), value = 0;
            for (int i = rpm.length - 1; i >= 0; i--) value = value * r + rpm[i];
            return value;
        }
    }
}
