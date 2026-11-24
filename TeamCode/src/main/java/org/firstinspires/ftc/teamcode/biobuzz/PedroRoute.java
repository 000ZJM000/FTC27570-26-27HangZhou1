package org.firstinspires.ftc.teamcode.biobuzz;

import android.content.res.AssetManager;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.curves.bezier.BezierCurve;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

final class PedroRoute {
    static final String ASSET = "pedro/two-group-route.json";

    final Pose start;
    final List<Leg> legs;

    private PedroRoute(Pose start, List<Leg> legs) {
        this.start = start;
        this.legs = legs;
    }

    static PedroRoute load(AssetManager assets) throws IOException, JSONException {
        return parse(readProject(assets));
    }

    static Pose loadStart(AssetManager assets) throws IOException, JSONException {
        return load(assets).start;
    }

    private static JSONObject readProject(AssetManager assets) throws IOException, JSONException {
        StringBuilder json = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(assets.open(ASSET), StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) json.append(line);
        }
        return new JSONObject(json.toString());
    }

    static PedroRoute parse(JSONObject project) throws JSONException {
        Pose start = point(project.getJSONObject("startPoint"));
        JSONArray lines = project.getJSONArray("lines"), sequence = project.getJSONArray("sequence");
        Map<String, JSONObject> byId = new HashMap<>();
        for (int i = 0; i < lines.length(); i++) {
            JSONObject line = lines.getJSONObject(i);
            byId.put(line.getString("id"), line);
        }
        List<Leg> legs = new ArrayList<>();
        Pose from = start;
        for (int i = 0; i < sequence.length(); i++) {
            JSONObject entry = sequence.getJSONObject(i);
            JSONObject line = byId.get(entry.getString("lineId"));
            if (!"path".equals(entry.getString("kind")) || line == null || !"atomic".equals(line.getString("kind"))) {
                throw new JSONException("不支持的路径段：" + (i + 1));
            }
            long before = line.getLong("waitBeforeMs"), after = line.getLong("waitAfterMs");
            if (before < 0 || after < 0) throw new JSONException("路径等待时间不能为负数");
            Pose end = point(line.getJSONObject("endPoint"));
            legs.add(new Leg(path(from, end, line), line.optString("name", "未命名路径"), before, after));
            from = end;
        }
        if (legs.isEmpty()) throw new JSONException("路径为空");
        // Match the Visualizer preview, which starts at the first path's heading.
        return new PedroRoute(start.withHeading(legs.get(0).path.heading(0)), legs);
    }

    private static Path path(Pose start, Pose end, JSONObject line) throws JSONException {
        JSONArray controls = line.getJSONArray("controlPoints");
        Path path;
        if (controls.length() == 0) {
            path = new AtomicPath(new Line(start, end));
        } else {
            Pose[] points = new Pose[controls.length() + 2];
            points[0] = start;
            for (int i = 0; i < controls.length(); i++) points[i + 1] = point(controls.getJSONObject(i));
            points[points.length - 1] = end;
            path = new AtomicPath(new BezierCurve(points));
        }
        JSONObject heading = line.getJSONObject("heading");
        switch (heading.getString("type")) {
            case "linear":
                return linearHeading(path, Math.toRadians(heading.getDouble("startDeg")),
                        Math.toRadians(heading.getDouble("endDeg")));
            case "constant":
                return path.constant(Math.toRadians(heading.getDouble("degrees")));
            case "tangential":
                return heading.optBoolean("reverse") ? path.reverseTangent() : path.tangent();
            default:
                throw new JSONException("不支持的航向模式：" + heading.getString("type"));
        }
    }

    private static Pose point(JSONObject point) throws JSONException {
        return new Pose(point.getDouble("x"), point.getDouble("y"));
    }

    private static Path linearHeading(Path path, double start, double end) {
        double turn = Maths.wrap(end - start);
        // Pedro 3.0.0's Line.pathCompletion returns the remaining fraction.
        return path.heading((curve, t) -> start + turn * (1 - curve.remainingDistance(t) / curve.length()));
    }

    static final class Leg {
        final Path path;
        final String name;
        final long waitBefore, waitAfter;

        Leg(Path path, String name, long waitBefore, long waitAfter) {
            this.path = path;
            this.name = name;
            this.waitBefore = waitBefore;
            this.waitAfter = waitAfter;
        }

        Path fromMeasuredPose(Pose start) {
            Pose end = path.endPose();
            Curve curve;
            if (path.curve instanceof BezierCurve) {
                List<Vector2D> points = new ArrayList<>(((BezierCurve) path.curve).getControlPoints());
                points.set(0, start.toVector2D());
                curve = new BezierCurve(points);
            } else {
                curve = new Line(start, end);
            }
            return linearHeading(new AtomicPath(curve), start.heading(), end.heading());
        }
    }
}
