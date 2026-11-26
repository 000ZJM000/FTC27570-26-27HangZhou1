# 云台旧版算法备份（2026-11-26）

- 备份时间：2026-11-26
- 备份原因：在保留原有电气配置、Pinpoint 定位、飞轮、吸球与其余自测校准程序不变的前提下，移植 DECODE 赛季云台控制算法（`DecodeTurret`）与全自动调参（`02B 云台 DECODE 全自动调参`）。

## 备份文件清单与原路径对照

| 备份文件名 | 原项目相对路径 |
| :--- | :--- |
| `Turret.java` | `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/subsystems/Turret.java` |
| `TurretModels.java` | `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/subsystems/TurretModels.java` |
| `Aiming.java` | `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/shooting/Aiming.java` |
| `BioBuzzTeleOp.java` | `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/opmode/BioBuzzTeleOp.java` |
| `BioBuzzPedroAuto.java` | `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/opmode/BioBuzzPedroAuto.java` |
| `CalibrationStore.java` | `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/config/CalibrationStore.java` |

## 回滚方法

如需完整回滚至移植前的原版云台算法：
1. 将本目录下的 6 个 `.java` 文件按上表原路径覆盖还原。
2. 删除新增文件（如不再需要）：
   - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/subsystems/TurretController.java`
   - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/subsystems/DecodeTurretConfig.java`
   - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/subsystems/DecodeTurret.java`
   - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/calibration/DecodeTurretAutoTuner.java`
3. 原有 `Turret` / `TurretModels` / `TurretCalibration (02)` / `YawCalibration (03)` / `TurretPidCalibration (05)` / `ShotFieldCalibration (06)` 在移植过程中均未被删除或破坏。
