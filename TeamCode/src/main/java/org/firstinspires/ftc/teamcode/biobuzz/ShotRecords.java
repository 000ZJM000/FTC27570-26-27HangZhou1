package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.util.RobotLog;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.List;

public final class ShotRecords {
    public static final String FILE = "shot_records.json";

    private ShotRecords() {}

    static List<ShotVectorField.PointData> points(CalibrationStore store) {
        List<ShotVectorField.PointData> points = new ShotVectorField(store, false).pointsForReview();
        points.addAll(new ShotVectorField(store, true).pointsForReview());
        return points;
    }

    public static boolean export(CalibrationStore store) {
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
            for (ShotVectorField.PointData point : points(store)) {
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
            return store.writeJson(FILE, new JSONObject().put("version", 2).put("kind", "biobuzz-shot-records")
                    .put("geometry", geometry).put("points", exported));
        } catch (JSONException malformed) {
            RobotLog.ww("BioBuzzShot", "export: %s", malformed.toString());
            return false;
        }
    }
}
