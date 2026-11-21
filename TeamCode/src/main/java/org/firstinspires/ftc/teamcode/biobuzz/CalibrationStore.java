package org.firstinspires.ftc.teamcode.biobuzz;

import android.content.Context;
import android.os.Environment;
import android.system.ErrnoException;
import android.system.Os;
import com.qualcomm.robotcore.util.RobotLog;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

public final class CalibrationStore {
    public static final String HOME = "turret_geometry";
    public static final String TURRET_PLANT = "turret_quick_identification_v1";
    public static final String TURRET_FRICTION_MAP = "turret_friction_by_angle_v2";
    public static final String TURRET_PID = "turret_single_pid_bayes_v1";
    public static final String TURRET_PID_STEADY_ERROR = "turret_pid_steady_error";
    public static final String TURRET_PID_HISTORY = "turret_pid_long_step_history_v2";
    public static final String TURRET_COUPLED_DYNAMICS = "turret_chassis_state_ff_v1";
    public static final String FLYWHEEL_DIRECTIONS = "flywheel_directions_v1";
    public static final String FLYWHEEL_SCALE = "flywheel_scale";
    public static final String FLYWHEEL_MODEL = "flywheel_model";
    public static final String FLYWHEEL_VOLTAGE_MODEL = "flywheel_voltage_curve";
    public static final String FLYWHEEL_FULL_POWER_MODEL = "flywheel_model_full_power_v1";
    public static final String FLYWHEEL_PID = "flywheel_pid";
    public static final String FLYWHEEL_VOLTAGE_PID = "flywheel_12v_hybrid_pid_v1";

    private static final String CALIBRATION = "calibration.json";
    private static final Object LOCK = new Object();
    private final File directory;
    private JSONObject data;

    public CalibrationStore(Context context) {
        File shared = new File(Environment.getExternalStorageDirectory(), "FIRST/biobuzz");
        directory = (shared.isDirectory() || shared.mkdirs()) && shared.canWrite()
                ? shared : new File(context.getFilesDir(), "biobuzz");
        synchronized (LOCK) {
            data = readCalibration();
            if (data != null) return;
            data = new JSONObject();
            JSONObject values = new JSONObject();
            try {
                data.put("version", 1).put("values", values);
                Map<String, ?> previous = context.getSharedPreferences("biobuzz_calibration_v1", Context.MODE_PRIVATE).getAll();
                for (Map.Entry<String, ?> entry : previous.entrySet()) {
                    if (!(entry.getValue() instanceof String)) continue;
                    try {
                        String raw = (String) entry.getValue();
                        double[] numbers = raw.isEmpty() ? new double[0]
                                : Arrays.stream(raw.split(",")).mapToDouble(Double::parseDouble).toArray();
                        if (Maths.finite(numbers)) values.put(entry.getKey(), array(numbers));
                    } catch (NumberFormatException ignored) {
                        // A malformed legacy entry need not discard the other calibrations.
                    }
                }
                Set<String> migratedStale = new LinkedHashSet<>();
                for (String key : new String[]{TURRET_PLANT, TURRET_FRICTION_MAP, TURRET_PID, TURRET_COUPLED_DYNAMICS}) {
                    if (values.has(key)) migratedStale.add(key);
                }
                data.put("turretStale", new JSONArray(migratedStale));
                if (!new File(directory, CALIBRATION).exists()) writeJson(CALIBRATION, data);
            } catch (JSONException malformed) {
                throw new IllegalStateException(malformed);
            }
        }
    }

    public File directory() { return directory; }

    private JSONObject readCalibration() {
        JSONObject document = readJson(CALIBRATION);
        return document != null && document.optInt("version") == 1
                && document.optJSONObject("values") != null ? document : null;
    }

    public JSONObject readJson(String name) {
        File source = new File(directory, name);
        if (source.length() > 4_000_000) return null;
        try (FileInputStream stream = new FileInputStream(source);
             ByteArrayOutputStream output = new ByteArrayOutputStream()) {
            byte[] buffer = new byte[8192];
            int count;
            while ((count = stream.read(buffer)) != -1) {
                if (output.size() + count > 4_000_000) return null;
                output.write(buffer, 0, count);
            }
            return new JSONObject(new String(output.toByteArray(), StandardCharsets.UTF_8));
        } catch (java.io.FileNotFoundException absent) {
            return null;
        } catch (IOException | JSONException unavailable) {
            RobotLog.ww("BioBuzzCalibration", "%s: %s", name, unavailable.toString());
            return null;
        }
    }

    public boolean writeJson(String name, JSONObject document) {
        synchronized (LOCK) {
            if (!directory.isDirectory() && !directory.mkdirs()) return false;
            File temporary = null;
            try {
                temporary = File.createTempFile(name + ".", ".tmp", directory);
                try (FileOutputStream stream = new FileOutputStream(temporary)) {
                    stream.write(document.toString(2).getBytes(StandardCharsets.UTF_8));
                    stream.getFD().sync();
                }
                Os.rename(temporary.getPath(), new File(directory, name).getPath());
                return true;
            } catch (IOException | JSONException | ErrnoException unavailable) {
                RobotLog.ww("BioBuzzCalibration", "%s: %s", name, unavailable.toString());
                return false;
            } finally {
                if (temporary != null && temporary.exists()) temporary.delete();
            }
        }
    }

    public double[] read(String key, double... fallback) {
        return readRecords(key, fallback.length, fallback.length, fallback.length, fallback);
    }

    public double[] readExact(String key, int count) {
        return readRecords(key, count, count, count, null);
    }

    public double[] readHistory(String key, int stride, int rows) {
        return readRecords(key, 0, stride * rows, stride, new double[0]);
    }

    private double[] readRecords(String key, int min, int max, int stride, double[] fallback) {
        JSONArray record = data.optJSONObject("values").optJSONArray(key);
        if (record == null || stride <= 0 || record.length() < min || record.length() > max
                || record.length() % stride != 0) return fallback;
        double[] values = new double[record.length()];
        for (int i = 0; i < values.length; i++) values[i] = record.optDouble(i);
        return Maths.finite(values) ? values : fallback;
    }

    public boolean save(String key, double... values) { return save(key, values, null, null); }

    public boolean saveTurretPid(double[] gains, double steadyError) {
        return gains.length == 3 && steadyError >= 0
                && save(TURRET_PID, gains, TURRET_PID_STEADY_ERROR, new double[]{steadyError});
    }

    boolean saveShot(String key, double[] values, String id, double battery, double pitch) {
        return save(key, values, "shot_meta_" + id, new double[]{battery, pitch});
    }

    private boolean save(String key, double[] values, String extra, double[] extraValues) {
        if (!Maths.finite(values) || extraValues != null && !Maths.finite(extraValues)) return false;
        if (FLYWHEEL_DIRECTIONS.equals(key)
                && (values.length != 2 || Math.abs(values[0]) != 1 || Math.abs(values[1]) != 1)) return false;
        synchronized (LOCK) {
            try {
                JSONObject latest = readCalibration();
                if (latest != null) data = latest;
                JSONObject changed = new JSONObject(data.toString());
                JSONObject records = changed.getJSONObject("values");
                Set<String> stale = staleKeys(changed);
                markDependents(key, values, stale);
                stale.remove(key);
                if (TURRET_PID.equals(key)) records.remove(TURRET_PID_STEADY_ERROR);
                if (extra != null) records.put(extra, array(extraValues));
                records.put(key, array(values));
                changed.put("turretStale", new JSONArray(stale));
                if (!writeJson(CALIBRATION, changed)) return false;
                data = changed;
                return true;
            } catch (JSONException malformed) {
                RobotLog.ww("BioBuzzCalibration", "%s: %s", key, malformed.toString());
                return false;
            }
        }
    }

    private void markDependents(String key, double[] values, Set<String> stale) {
        double[] old = readExact(key, values.length);
        boolean home = HOME.equals(key) && values.length == 4 && old != null;
        boolean signs = home && (values[0] != old[0] || values[1] != old[1]);
        boolean span = home && Math.abs(values[2] - old[2]) > Math.toRadians(2);
        boolean plant = TURRET_PLANT.equals(key) && !Arrays.equals(values, old);
        boolean friction = TURRET_FRICTION_MAP.equals(key) && !Arrays.equals(values, old);
        if (signs && has(TURRET_PLANT)) stale.add(TURRET_PLANT);
        if ((signs || span || plant) && has(TURRET_FRICTION_MAP)) stale.add(TURRET_FRICTION_MAP);
        if (signs || span || plant || friction) {
            if (has(TURRET_PID)) stale.add(TURRET_PID);
            if (has(TURRET_COUPLED_DYNAMICS)) stale.add(TURRET_COUPLED_DYNAMICS);
        }
    }

    static JSONArray array(double[] values) throws JSONException {
        JSONArray array = new JSONArray();
        for (double value : values) array.put(value);
        return array;
    }

    public boolean has(String key) { return data.optJSONObject("values").has(key); }
    public boolean turretStale() { return !staleKeys(data).isEmpty(); }

    private static Set<String> staleKeys(JSONObject document) {
        Set<String> keys = new LinkedHashSet<>();
        JSONArray stored = document.optJSONArray("turretStale");
        if (stored != null) for (int i = 0; i < stored.length(); i++) keys.add(stored.optString(i));
        return keys;
    }

    public boolean clearLegacyShotPoints() {
        synchronized (LOCK) {
            try {
                JSONObject latest = readCalibration();
                JSONObject changed = new JSONObject((latest != null ? latest : data).toString());
                JSONObject records = changed.getJSONObject("values");
                records.remove("shot_vector_refinements_left_v1");
                records.remove("shot_vector_refinements_right_v1");
                if (!writeJson(CALIBRATION, changed)) return false;
                data = changed;
                return true;
            } catch (JSONException malformed) {
                return false;
            }
        }
    }
}
