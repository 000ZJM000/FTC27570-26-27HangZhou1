package org.firstinspires.ftc.teamcode.biobuzz;

import android.content.Context;
import android.content.SharedPreferences;
import java.util.Arrays;

public final class CalibrationStore {
    public static final String HOME = "turret_geometry";
    public static final String TURRET_PLANT = "turret_quick_identification_v1";
    public static final String TURRET_FRICTION_MAP = "turret_friction_by_angle_v2";
    public static final String TURRET_PID = "turret_single_pid_bayes_v1";
    public static final String TURRET_PID_HISTORY = "turret_pid_long_step_history_v2";
    public static final String TURRET_COUPLED_DYNAMICS = "turret_chassis_state_ff_v1";
    public static final String FLYWHEEL_SCALE = "flywheel_scale";
    public static final String FLYWHEEL_MODEL = "flywheel_model";
    public static final String FLYWHEEL_FULL_POWER_MODEL = "flywheel_model_full_power_v1";
    public static final String FLYWHEEL_PID = "flywheel_pid";
    public static final String FLYWHEEL_PID_HISTORY = "flywheel_pid_full_power_history_v2";

    private final SharedPreferences prefs;

    public CalibrationStore(Context context) {
        prefs = context.getSharedPreferences("biobuzz_calibration_v1", Context.MODE_PRIVATE);
    }

    public double[] read(String key, double... fallback) {
        return readRecords(key, 1, fallback.length, fallback.length, fallback);
    }

    public double[] readExact(String key, int count) {
        return readRecords(key, count, count, count, null);
    }

    public double[] readHistory(String key, int stride, int rows) {
        return readRecords(key, 0, stride * rows, stride, new double[0]);
    }

    private double[] readRecords(String key, int minimum, int maximum, int stride, double[] fallback) {
        try {
            String raw = prefs.getString(key, "");
            if (raw.isEmpty() || stride <= 0) return fallback;
            String[] fields = raw.split(",");
            if (fields.length < minimum || fields.length > maximum || fields.length % stride != 0) return fallback;
            double[] values = new double[fields.length];
            for (int i = 0; i < fields.length; i++) values[i] = Double.parseDouble(fields[i]);
            return Maths.finite(values) ? values : fallback;
        } catch (IllegalArgumentException | ClassCastException ignored) {
            return fallback;
        }
    }

    public boolean save(String key, double... values) {
        if (!Maths.finite(values)) return false;
        StringBuilder result = new StringBuilder();
        for (double value : values) {
            if (result.length() > 0) result.append(',');
            result.append(value);
        }
        SharedPreferences.Editor edit = prefs.edit().putString(key, result.toString());
        if (HOME.equals(key) && values.length == 4) {
            double[] previous = readExact(HOME, 4);
            if (previous == null || values[0] != previous[0] || values[1] != previous[1]) {
                edit.remove(TURRET_PLANT).remove(TURRET_FRICTION_MAP);
                clearTurretTuning(edit);
            } else if (Math.abs(values[2] - previous[2]) > Math.toRadians(2)) {
                edit.remove(TURRET_FRICTION_MAP);
                clearTurretTuning(edit);
            }
        }
        if ((TURRET_PLANT.equals(key) || TURRET_FRICTION_MAP.equals(key))
                && !Arrays.equals(values, readExact(key, values.length))) {
            if (TURRET_PLANT.equals(key)) edit.remove(TURRET_FRICTION_MAP);
            clearTurretTuning(edit);
        }
        if (FLYWHEEL_FULL_POWER_MODEL.equals(key) && fullPowerModelChanged(values)) {
            edit.remove(FLYWHEEL_PID_HISTORY);
        }
        if (FLYWHEEL_SCALE.equals(key) && values.length == 1
                && Math.abs(values[0] - read(FLYWHEEL_SCALE, 0)[0]) > 0.01 * values[0]) {
            edit.remove(FLYWHEEL_PID_HISTORY);
        }
        return edit.commit();
    }

    public boolean has(String key) { return prefs.contains(key); }

    private static void clearTurretTuning(SharedPreferences.Editor edit) {
        edit.remove(TURRET_PID).remove(TURRET_PID_HISTORY).remove(TURRET_COUPLED_DYNAMICS);
    }

    private boolean fullPowerModelChanged(double[] model) {
        if (model.length != 4) return true;
        double[] previous = read(FLYWHEEL_FULL_POWER_MODEL, 0, 0, 0, 0);
        return Math.abs(model[0] - previous[0]) > 0.015 * model[0]
                || Math.abs(model[3] - previous[3]) > 0.015 * model[3]
                || Math.abs(model[1] - previous[1]) > 0.02;
    }
}
