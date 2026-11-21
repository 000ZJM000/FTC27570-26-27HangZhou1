package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.util.RobotLog;

final class HardwareStop {
    private HardwareStop() {}

    static void attempt(String device, Runnable stop) {
        try {
            stop.run();
        } catch (RuntimeException unavailable) {
            RobotLog.ww("BioBuzzStop", "%s: %s", device, unavailable.toString());
        }
    }
}
