package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

public final class BatteryVoltage {
    private static final double SAMPLE_PERIOD = 0.20;
    public static final double NOMINAL_VOLTS = 12.0;
    private static final double LOW_VOLTS = 10.2;
    static final double LOW_DRIVE_LIMIT = 0.80;

    private final List<VoltageSensor> sensors;
    private double nextSample = Double.NEGATIVE_INFINITY;
    private double volts = NOMINAL_VOLTS;
    private boolean measured;

    public BatteryVoltage(HardwareMap map) {
        sensors = map.getAll(VoltageSensor.class);
    }

    public void update(double now) {
        if (now < nextSample) return;
        nextSample = now + SAMPLE_PERIOD;
        double highest = 0;
        for (VoltageSensor sensor : sensors) {
            try {
                double reading = sensor.getVoltage();
                if (Double.isFinite(reading)) highest = Math.max(highest, reading);
            } catch (RuntimeException unavailable) {
                // Check any other Hub that may still be responding.
            }
        }
        measured = highest > 0;
        volts = measured ? Maths.clip(highest, 9.0, 14.5) : NOMINAL_VOLTS;
    }

    public double volts() { return volts; }
    public boolean measured() { return measured; }
    public double duty(double nominal, double limit) {
        return Maths.clip(nominal * NOMINAL_VOLTS / volts, -limit, limit);
    }
    public double nominal(double duty) { return duty * volts / NOMINAL_VOLTS; }
    boolean low() { return measured && volts < LOW_VOLTS; }
}
