/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */
package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.Register;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/** goBILDA's Pinpoint example with robot-relative driving and fault logging. */
@TeleOp(name = "goBILDA Pinpoint Example", group = "BIOBUZZ 诊断")
public final class SensorGoBildaPinpointExample extends LinearOpMode {
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
                    // Restore the full read set if the previous OpMode used a reduced one.
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
            telemetry.addLine("保持车静止，让 IMU 校零。START 后可用摇杆开车。");
            telemetry.addData("固件", "V%d", version);
            telemetry.addData("X / Y 轮偏移", "%+.2f / %+.2f mm",
                    -RobotConfig.X_POD_RIGHT_MM, RobotConfig.Y_POD_FORWARD_MM);
            telemetry.addData("X / Y 编码器方向", "%s / %s", RobotConfig.X_POD_DIRECTION, RobotConfig.Y_POD_DIRECTION);
            telemetry.update();
            waitForStart();
            resetRuntime();

            boolean lastA = gamepad1.a, lastB = gamepad1.b;
            double lastTime = Maths.time();
            int badReads = 0;
            String lastFault = null;
            while (opModeIsActive()) {
                if (drive != null) drive.robot(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                boolean a = gamepad1.a, b = gamepad1.b;
                boolean reset = a && !lastA, recalibrate = b && !lastB;
                lastA = a;
                lastB = b;
                double now = Maths.time();
                double elapsed = now - lastTime;
                lastTime = now;
                String fault = null;
                try {
                    if (reset) odo.resetPosAndIMU();
                    else if (recalibrate) odo.recalibrateIMU();
                    odo.update();
                    DeviceStatus status = odo.getDeviceStatus();
                    if (status == DeviceStatus.FAULT_BAD_READ) badReads++;
                    RearStatusLed.State light = lightFor(status);
                    if (light == RearStatusLed.State.FAULT) fault = status.name();
                    lights.update(light);
                    showReadings(odo);
                    telemetry.addData("Pinpoint V" + version, "%s / 坏读累计 %d", status, badReads);
                    telemetry.addData("Pinpoint / OpMode 刷新率", "%.0f / %.0f Hz",
                            odo.getFrequency(), elapsed > 0 ? 1 / elapsed : 0);
                } catch (RuntimeException failure) {
                    lights.update(RearStatusLed.State.FAULT);
                    fault = "Pinpoint 读写失败 " + failure;
                }
                if (fault != null && !fault.equals(lastFault)) logFault(fault);
                lastFault = fault;
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

    private static RearStatusLed.State lightFor(DeviceStatus status) {
        if (status == DeviceStatus.READY) return RearStatusLed.State.CAL_DONE;
        if (status == DeviceStatus.CALIBRATING || status == DeviceStatus.NOT_READY) return RearStatusLed.State.CAL_WAIT;
        return RearStatusLed.State.FAULT;
    }
}
