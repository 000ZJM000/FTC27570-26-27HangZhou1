package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.biobuzz.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.PinpointPose;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

@TeleOp(name = "03 云台专项 12V联动补偿", group = "BIOBUZZ 自测")
public final class YawCalibration extends CalibrationOp {
    @Override String intro() { return "轮子落地并腾出空地。用底盘转动测试云台反向补偿。"; }

    @Override void calibrate() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, store, battery);
        try {
            if (!turret.calibrated() || !turret.plantIdentified()) {
                telemetry.log().add("先运行 02，保存云台行程和动力学。");
                return;
            }
            turret.teachingMode(true);
            if (!confirm("把云台摆到最右端，停稳后 A 记零位。")) return;
            turret.homeAtRight();
            turret.teachingMode(false);
            if (!turret.encoderReady()
                    || !confirm("轮子落地、清空周围并保持底盘静止。A 初始化 Pinpoint 后开始双向联动。")) return;
            PinpointPose pose = new PinpointPose(hardwareMap, battery);
            if (!initializePinpoint(pose)) {
                telemetry.log().add("Pinpoint 航向未就绪：%s。", pose.status);
                return;
            }
            Drive drive = new Drive(hardwareMap);
            boolean saved = new TurretYawCalibration(this).run(drive, pose, turret);
            telemetry.log().add(saved ? "双向联动结果已验收保存" : "联动未保存，沿用之前的参数");
        } finally {
            turret.teachingMode(false);
        }
    }
}
