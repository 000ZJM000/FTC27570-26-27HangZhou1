package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.biobuzz.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.PinpointPose;

@TeleOp(name = "03 自测云台反转补偿", group = "BIOBUZZ 自测")
public final class YawCalibration extends CalibrationOp {
    @Override String intro() { return "轮子落地并腾出空地。用底盘转动测试云台反向补偿。"; }

    @Override void calibrate() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);
        PinpointPose pose = new PinpointPose(hardwareMap);
        try {
            do {
                if (!confirm("保持底盘静止，A 初始化 Pinpoint。")) return;
                if (pose.startImu()) {
                    runFor(2.0, (t, dt) -> { pose.update(); return !pose.headingValid; });
                    if (pose.headingValid) break;
                }
                telemetry.log().add("Pinpoint 航向未就绪：%s。", pose.status);
            } while (opModeIsActive() && !motionAborted());
            if (opModeIsActive() && !motionAborted()) new TurretYawCalibration(this).run(drive, pose);
        } finally {
            drive.stop();
        }
    }
}
