package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.vision.KIRIN.STM32LedModule;

/** State-only writer for the rear KIRIN strip. The module renders each animation locally. */
public final class RearStatusLed {
    private static final double STABLE_FOR_SECONDS = 0.10, WRITE_INTERVAL_SECONDS = 0.28;
    private final STM32LedModule module;
    private State shown, pending;
    private double pendingSince, lastWrite;

    public enum State {
        INIT, DRIVE, AIMED, AIMED_LOCKED, UNLOCKED, INTAKE, REVERSE, NO_FIRE,
        CAL_WAIT, CAL_RUNNING, CAL_DONE, FAULT, OFF
    }

    private RearStatusLed(STM32LedModule module) { this.module = module; }

    public static RearStatusLed open(HardwareMap map) {
        try {
            STM32LedModule module = map.tryGet(STM32LedModule.class, RobotConfig.REAR_STATUS_LED);
            if (module != null) module.assignId('A');
            return new RearStatusLed(module);
        } catch (RuntimeException unavailable) {
            return new RearStatusLed(null);
        }
    }

    public boolean configured() { return module != null; }

    /** Use at program and calibration phase boundaries. Identical states send no writes. */
    public void show(State state) {
        if (module == null || state == shown) return;
        pending = shown = state;
        switch (state) {
            case INIT: module.setBreathMode(true, 40, 1.35, 55, 0x249BFF); break;
            case DRIVE: module.setSolidColor(0x165EFF, 45); break;
            case AIMED: module.setSolidColor(0x00E454, 72); break;
            case AIMED_LOCKED: module.setSolidColor(0x00E5FF, 72); break;
            case UNLOCKED: module.setSolidColor(0xFFD000, 68); break;
            case INTAKE: module.setWaveMode(0x02CE94, 0x067471, 44, 68); break;
            case REVERSE: module.setSolidColor(0xEA27CF, 68); break;
            case NO_FIRE: module.setSolidColor(0xFF7100, 74); break;
            case CAL_WAIT: module.setSolidColor(0xF0B42A, 55); break;
            case CAL_RUNNING: module.setWaveMode(0x496FFF, 0x7D30CC, 42, 70); break;
            case CAL_DONE: module.setSolidColor(0x9BEDE2, 62); break;
            case FAULT: module.setSolidColor(0xFF2312, 72); break;
            case OFF: module.turnOff(); break;
        }
        lastWrite = Maths.time();
    }

    /** Called by TeleOp every loop; filters short state changes and limits the command rate. */
    public void update(State desired) {
        if (module == null) return;
        double now = Maths.time();
        if (desired != pending) {
            pending = desired;
            pendingSince = now;
        }
        if (desired != shown && now - pendingSince >= STABLE_FOR_SECONDS
                && now - lastWrite >= WRITE_INTERVAL_SECONDS) show(desired);
    }

    public void stop() { show(State.OFF); }
}
