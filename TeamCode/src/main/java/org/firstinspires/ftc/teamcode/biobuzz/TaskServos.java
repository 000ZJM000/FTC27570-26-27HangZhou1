package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

final class TaskServos {
    private final Servo right, left;
    private boolean switched;

    TaskServos(HardwareMap map) {
        right = map.tryGet(Servo.class, RobotConfig.TASK_RIGHT_SERVO);
        left = map.tryGet(Servo.class, RobotConfig.TASK_LEFT_SERVO);
        if (right != null) right.setPosition(RobotConfig.FR_HOME);
        if (left != null) left.setPosition(RobotConfig.FL_HOME);
    }

    void toggle() {
        setLowered(!switched);
    }

    void setLowered(boolean lowered) {
        if (!configured() || switched == lowered) return;
        right.setPosition(lowered ? RobotConfig.FR_OTHER : RobotConfig.FR_HOME);
        left.setPosition(lowered ? RobotConfig.FL_OTHER : RobotConfig.FL_HOME);
        switched = lowered;
    }

    boolean configured() { return right != null && left != null; }

    void stop() {
        if (right != null) HardwareStop.attempt(RobotConfig.TASK_RIGHT_SERVO, () -> right.setPosition(RobotConfig.FR_HOME));
        if (left != null) HardwareStop.attempt(RobotConfig.TASK_LEFT_SERVO, () -> left.setPosition(RobotConfig.FL_HOME));
        switched = false;
    }

    String status() {
        if (right == null && left == null) return "未配置 FR、FL";
        if (right == null) return "未配置 FR";
        if (left == null) return "未配置 FL";
        return switched ? "0.00 / 1.00" : "0.94 / 0.06";
    }
}
