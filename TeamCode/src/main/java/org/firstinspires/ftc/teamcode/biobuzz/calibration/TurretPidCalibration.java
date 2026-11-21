package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

@TeleOp(name = "05 云台专项 12V PID·回转", group = "BIOBUZZ 自测")
public final class TurretPidCalibration extends CalibrationOp {
    private TurretPidTuning tuning;

    @Override String intro() { return "使用已有行程和摩擦。支持 30 秒 PID 快检，或完整搜索及双向回转。"; }

    @Override void calibrate() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, store, battery);
        try {
            if (!turret.calibrated()) {
                telemetry.log().add("尚无云台方向和行程，请先运行 02 云台全套。");
                return;
            }
            turret.teachingMode(true);
            boolean homed = false;
            while (confirm("把云台摆到最右标记，停稳后按 A 记零位。")) {
                turret.homeAtRight();
                if (turret.encoderReady()) {
                    homed = true;
                    break;
                }
            }
            if (!homed || !opModeIsActive()) return;
            turret.teachingMode(false);
            tuning = new TurretPidTuning(this);
            if (confirm("轮子落地并清空周围。A 运行 30 秒快检（含底盘双向回转），B 进入完整搜索选项。")) {
                tuning.quickCheck(turret);
            } else if (confirm("清空云台周围。A 开始 8–12 轮搜索及留出复测，B 结束。")) {
                tuning.tune(turret, null, null);
            }
        } finally {
            turret.teachingMode(false);
        }
    }

    @Override void showResults() {
        if (tuning != null) tuning.showResults();
    }
}
