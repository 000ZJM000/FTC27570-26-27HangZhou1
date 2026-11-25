package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.Register;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.biobuzz.config.RobotConfig;
import org.firstinspires.ftc.teamcode.biobuzz.core.Maths;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Drive;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.RearStatusLed;

/** goBILDA's Pinpoint diagnostic TeleOp with robot-relative driving and fault logging. */
@TeleOp(name = "goBILDA Pinpoint Example", group = "BIOBUZZ 诊断")
public final class PinpointDiagnosticOp extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        RearStatusLed lights = RearStatusLed.open(hardwareMap);
        Drive drive = null;
        telemetry.log().setCapacity(5);
        try {
            lights.show(RearStatusLed.State.INIT);
            GoBildaPinpointDriver odo = hardwareMap.tryGet(GoBildaPinpointDriver.class, RobotConfig.PINPOINT);
            if (odo == null) {
                lights.show(RearStatusLed.State.FAULT);
                logFault("找不到 " + RobotConfig.PINPOINT);
                telemetry.update();
                waitForStart();
                return;
            }
            int version;
            try {
                version = RobotConfig.configurePinpoint(odo);
                if (version >= 3) {
                    odo.setBulkReadScope(Register.DEVICE_STATUS, Register.LOOP_TIME,
                            Register.X_ENCODER_VALUE, Register.Y_ENCODER_VALUE, Register.X_POSITION,
                            Register.Y_POSITION, Register.H_ORIENTATION, Register.X_VELOCITY,
                            Register.Y_VELOCITY, Register.H_VELOCITY);
                }
                odo.resetPosAndIMU();
            } catch (RuntimeException failure) {
                lights.show(RearStatusLed.State.FAULT);
                logFault("Pinpoint 初始化失败 " + failure);
                telemetry.update();
                waitForStart();
                return;
            }
            try {
                drive = new Drive(hardwareMap);
            } catch (RuntimeException failure) {
                logFault("底盘不可用，可手推测试 " + failure);
            }
            IMU hubImu = hardwareMap.tryGet(IMU.class, RobotConfig.HUB_IMU);
            LogoFacingDirection hubLogo = resolvedLogoDir();
            UsbFacingDirection hubUsb = resolvedUsbDir();
            String hubError = initHubImu(hubImu, hubLogo, hubUsb);
            if (hubError != null) logFault("Hub IMU: " + hubError);

            telemetry.addLine("保持车静止，让 IMU 校零。START 后可用摇杆开车。");
            telemetry.addData("固件", "V%d", version);
            telemetry.addData("X / Y 轮偏移", "%+.2f / %+.2f mm",
                    -RobotConfig.X_POD_RIGHT_MM, RobotConfig.Y_POD_FORWARD_MM);
            telemetry.addData("X / Y 编码器方向", "%s / %s", RobotConfig.X_POD_DIRECTION, RobotConfig.Y_POD_DIRECTION);
            telemetry.addData("Hub IMU 朝向", hubError == null
                    ? String.format("LOGO=%s / USB=%s", hubLogo, hubUsb) : hubError);
            telemetry.update();
            waitForStart();
            resetRuntime();

            boolean lastA = gamepad1.a, lastB = gamepad1.b;
            double lastTime = Maths.time();
            double hubYawZero = Double.NaN;
            boolean pendingHubZero = true;
            int badReads = 0;
            String lastFault = null;
            while (opModeIsActive()) {
                if (drive != null) drive.robot(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                boolean a = gamepad1.a, b = gamepad1.b;
                boolean reset = a && !lastA, recalibrate = b && !lastB;
                lastA = a;
                lastB = b;
                if (reset || recalibrate) pendingHubZero = true;

                LogoFacingDirection desiredLogo = resolvedLogoDir();
                UsbFacingDirection desiredUsb = resolvedUsbDir();
                if (hubImu != null && (desiredLogo != hubLogo || desiredUsb != hubUsb)) {
                    hubLogo = desiredLogo;
                    hubUsb = desiredUsb;
                    hubError = initHubImu(hubImu, hubLogo, hubUsb);
                    hubYawZero = Double.NaN;
                    pendingHubZero = true;
                }

                double now = Maths.time();
                double elapsed = now - lastTime;
                lastTime = now;
                String fault = null;
                boolean pinpointHeadingReady = false;
                double pinpointHeading = Double.NaN;
                try {
                    if (reset) odo.resetPosAndIMU();
                    else if (recalibrate) odo.recalibrateIMU();
                    odo.update();
                    DeviceStatus status = odo.getDeviceStatus();
                    if (status == DeviceStatus.FAULT_BAD_READ) badReads++;
                    RearStatusLed.State light = lightFor(status);
                    if (light == RearStatusLed.State.FAULT) fault = status.name();
                    lights.update(light);
                    if (status == DeviceStatus.READY) {
                        pinpointHeading = odo.getHeading(UnnormalizedAngleUnit.RADIANS);
                        pinpointHeadingReady = Double.isFinite(pinpointHeading);
                    }
                    showReadings(odo);
                    telemetry.addData("Pinpoint V" + version, "%s / 坏读累计 %d", status, badReads);
                    telemetry.addData("Pinpoint / OpMode 刷新率", "%.0f / %.0f Hz",
                            odo.getFrequency(), elapsed > 0 ? 1 / elapsed : 0);
                } catch (RuntimeException failure) {
                    lights.update(RearStatusLed.State.FAULT);
                    fault = "Pinpoint 读写失败 " + failure;
                }

                if (hubImu != null && hubError == null) {
                    try {
                        double rawHubYaw = hubImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                        AngularVelocity velocity = hubImu.getRobotAngularVelocity(AngleUnit.DEGREES);
                        double hubRateDeg = velocity != null ? velocity.zRotationRate : 0;
                        if (Double.isFinite(rawHubYaw)) {
                            if (pendingHubZero && pinpointHeadingReady) {
                                hubYawZero = Maths.wrap(rawHubYaw - Maths.wrap(pinpointHeading));
                                pendingHubZero = false;
                            } else if (!Double.isFinite(hubYawZero)) {
                                hubYawZero = rawHubYaw;
                            }
                            double hubYaw = Maths.wrap(rawHubYaw - hubYawZero);
                            double diff = pinpointHeadingReady ? Maths.wrap(hubYaw - Maths.wrap(pinpointHeading)) : Double.NaN;
                            telemetry.addData("Hub IMU 朝向", "LOGO=%s / USB=%s", hubLogo, hubUsb);
                            telemetry.addData("Hub IMU 航向 / 转向 / 差值", "%+.1f° / %+.1f°/s / 差 %+.1f°",
                                    Math.toDegrees(hubYaw), hubRateDeg, Math.toDegrees(diff));
                        }
                    } catch (RuntimeException failure) {
                        telemetry.addData("Hub IMU", "读取失败 %s", failure.getClass().getSimpleName());
                    }
                } else {
                    telemetry.addData("Hub IMU", hubError != null ? hubError : "未配置");
                }

                if (fault != null && !fault.equals(lastFault)) logFault(fault);
                lastFault = fault;
                telemetry.addLine("原地旋转核对 Pinpoint 与 Hub IMU 航向正负与度数是否一致");
                telemetry.addLine("车头模式 | 左摇杆平移 / 右摇杆横向转向");
                telemetry.addLine("停稳后 A 位姿归零并重校 IMU / B 仅重校 IMU");
                telemetry.update();
                sleep(15);
            }
        } finally {
            try {
                if (drive != null) drive.stop();
            } finally {
                lights.stop();
            }
        }
    }

    private void logFault(String fault) {
        telemetry.log().add("%.2f s  %s", getRuntime(), fault);
        RobotLog.ww("PinpointExample", "%s", fault);
    }

    private void showReadings(GoBildaPinpointDriver odo) {
        double x = odo.getPosX(DistanceUnit.MM), y = odo.getPosY(DistanceUnit.MM);
        double heading = odo.getHeading(UnnormalizedAngleUnit.RADIANS);
        telemetry.addData("位置 X 前 / Y 左 / 航向", "%+.1f mm / %+.1f mm / %+.1f°",
                x, y, Math.toDegrees(Maths.wrap(heading)));
        telemetry.addData("前 / 右 / 连续航向", "%+.2f 格 / %+.2f 格 / %+.1f°",
                x / RobotConfig.TILE_MM, -y / RobotConfig.TILE_MM, Math.toDegrees(heading));
        telemetry.addData("速度 X / Y / 转向", "%+.0f / %+.0f mm/s / %+.1f°/s",
                odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM),
                odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("里程轮 X / Y", "%d / %d ticks", odo.getEncoderX(), odo.getEncoderY());
    }

    private static LogoFacingDirection resolvedLogoDir() {
        return RobotConfig.HUB_IMU_LOGO_DIR != null ? RobotConfig.HUB_IMU_LOGO_DIR : LogoFacingDirection.RIGHT;
    }

    private static UsbFacingDirection resolvedUsbDir() {
        return RobotConfig.HUB_IMU_USB_DIR != null ? RobotConfig.HUB_IMU_USB_DIR : UsbFacingDirection.UP;
    }

    private static String initHubImu(IMU hubImu, LogoFacingDirection logo, UsbFacingDirection usb) {
        if (hubImu == null) return "未配置 " + RobotConfig.HUB_IMU;
        try {
            boolean ok = hubImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
            return ok ? null : "初始化返回 false";
        } catch (RuntimeException failure) {
            return failure instanceof IllegalArgumentException
                    ? "朝向配置冲突 (" + logo + "/" + usb + ")"
                    : "初始化失败 " + failure.getClass().getSimpleName();
        }
    }

    private static RearStatusLed.State lightFor(DeviceStatus status) {
        if (status == DeviceStatus.READY) return RearStatusLed.State.CAL_DONE;
        if (status == DeviceStatus.CALIBRATING || status == DeviceStatus.NOT_READY) return RearStatusLed.State.CAL_WAIT;
        return RearStatusLed.State.FAULT;
    }
}
