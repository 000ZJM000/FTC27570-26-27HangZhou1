package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedHashMap;
import java.util.Map;

public final class ControlLoop implements AutoCloseable {
    public static final double PERIOD = 0.015;

    private final Map<LynxModule, LynxModule.BulkCachingMode> previousModes = new LinkedHashMap<>();

    public ControlLoop(HardwareMap hardware) {
        try {
            for (LynxModule module : hardware.getAll(LynxModule.class)) {
                previousModes.put(module, module.getBulkCachingMode());
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        } catch (RuntimeException failure) {
            close();
            throw failure;
        }
    }

    public double beginFrame() {
        double now = Maths.time();
        for (LynxModule module : previousModes.keySet()) module.clearBulkCache();
        return now;
    }

    public long remainingMillis(double frameStart) {
        return Math.max(0, (long) Math.ceil(1000 * (PERIOD - (Maths.time() - frameStart))));
    }

    @Override public void close() {
        for (Map.Entry<LynxModule, LynxModule.BulkCachingMode> entry : previousModes.entrySet()) {
            LynxModule module = entry.getKey();
            HardwareStop.attempt("Lynx bulk cache", () -> {
                module.clearBulkCache();
                module.setBulkCachingMode(entry.getValue());
            });
        }
        previousModes.clear();
    }
}
