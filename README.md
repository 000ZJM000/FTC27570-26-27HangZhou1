# FTC27570-26-27HangZhou1

FTC 27570 在 2026–2027 BIOBUZZ 赛季使用的机器人控制程序。项目基于 FIRST FTC SDK 12.0，主控程序集成麦克纳姆底盘、goBILDA Pinpoint 双里程轮定位、云台自动瞄准、双飞轮与供弹联锁，以及 KIRIN 车尾状态灯。

- **比赛主控** · Driver Station → TeleOp → **BIOBUZZ 360 云台 + 无头**
- **标定地图跑打** · Driver Station → TeleOp → **BIOBUZZ → Biobuzz角落出发**
- **硬件自测** · Driver Station → TeleOp → **BIOBUZZ 自测**，依次运行 **02 → 03 → 04 → 05**
- **射击标定与查看** · 完成硬件自测后运行 **06 半场射击校准**，通过 **07 校准数据查看** 浏览记录
- **定位诊断** · Driver Station → TeleOp → **BIOBUZZ 诊断 → goBILDA Pinpoint Example**
- **离线拟合** · 下载完整仓库后，在浏览器打开 [射击拟合工作台/index.html](射击拟合工作台/index.html)
- **详细接线、坐标和自测流程** · [BIOBUZZ 上车指南](BIOBUZZ-上车指南.md)

## 构建与部署

1. 用 Android Studio 打开仓库根目录。Gradle JDK 使用 **JDK 17 或兼容的更高版本**，在 SDK Manager 安装 **Android SDK Platform 30**。仓库已包含 **Gradle 9.1.0 Wrapper**，项目使用 **Android Gradle Plugin 8.13.2**。
2. 等待 Gradle Sync 完成，选择 `TeamCode` 运行配置，将 **Control Hub** 连接为 Android 部署目标并点击 **Run**。
3. 在 Driver Station 连接 Robot Controller，在 **Robot Configuration** 中确认配置名称与下方表格一致，然后进入自测或比赛主控。

也可以仅构建调试 APK。在仓库根目录运行对应系统的命令。

| 系统 | 命令 |
| --- | --- |
| Windows PowerShell | `.\gradlew.bat :TeamCode:assembleDebug` |
| macOS / Linux | `bash ./gradlew :TeamCode:assembleDebug` |

APK 输出到 `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`。

## Robot Configuration

设备名称区分大小写。端口号从 0 开始。

| 机构 | 配置名称 | Hub 与端口 |
| --- | --- | --- |
| 底盘左前、左后 | `m-1`、`m-2` | Expansion Hub 电机 0、1 |
| 底盘右前、右后 | `m-3`、`m-4` | Control Hub 电机 0、1 |
| 吸取与供弹 | `m-5` | Control Hub 电机 2 |
| 云台电机 | `m-6` | Control Hub 电机 3 |
| 云台外置编码器 | `m-1` | Expansion Hub 编码器 0，与左前轮电机使用同一配置名称 |
| 左右飞轮 | `m-7`、`m-8` | Expansion Hub 电机 2、3，转速反馈接编码器 3 |
| 吸取舵机 | `SL`、`SR` | Expansion Hub 舵机 0、Control Hub 舵机 5 |
| 挡弹与俯仰舵机 | `SB`、`ST` | Control Hub 舵机 0、Expansion Hub 舵机 1 |
| goBILDA Pinpoint | `pinpoint` | Expansion Hub 2 I²C Bus 1，接两个里程轮 |
| KIRIN V3 尾灯（可选） | `LED` | 接线所用 I²C 总线，设备类型选 **STM32 Smart LED Module V3** |

飞轮方向以 04 保存的结果为准。Pinpoint 使用 FTC SDK 的 goBILDA 4-Bar 里程轮预设，机械尺寸与安装方向集中在 `RobotConfig.java`。

## 第一次上车

清空仓内和供弹通道，给底盘和云台留出运动空间。在 Driver Station 启动各自测后，按 **A** 确认，按 **B** 跳过提示或停止当前动作，按 **STOP** 退出 OpMode。

| 顺序 | Driver Station 名称 | 自测内容 |
| --- | --- | --- |
| 02 | **02 自测云台 行程 摩擦 刹车模型** | 教学云台方向和机械限位，测量摩擦和刹车模型 |
| 03 | **03 自测云台反转补偿** | 轮子落地，测量底盘转动对云台的影响并拟合补偿 |
| 04 | **04 自测飞轮 转速和 PID** | 标定每转计数，扫 17 档功率，再按实测范围复测 7–8 个 PID 转速点 |
| 05 | **05 自测云台 PID 与回转** | 整定云台 PID。行程达到 270° 时，新参数要通过双向回转才可选用 |

04 和 05 复测后会并排显示旧 PID 和通过复测的最佳新 PID。按 **A** 保存新值，按 **B** 保留旧值。

标定结果保存在 Robot Controller 的应用数据中。换 Hub 或清除应用数据后需要重新标定。02 的动力学或摩擦表改变后，需要重做 03 和 05。重新教学行程时，若编码器或电机旋向改变，需要重测动力学和摩擦表。行程变化超过 2° 时，需要重测摩擦表并重做 03 和 05。

主程序 INIT 会显示实际加载的 PID 数值、02 刹车模型与摩擦表，以及 03 左右补偿。运行中按住 **Back** 可以查看目标角、PID 设定角、行程角、角速度和可用功率。自动瞄准与跨接缝回转共用已加载的角度 PID。

## 射击数据标定与查看

完成 02 的云台行程与正前方向、04 的飞轮 RPM 标定后，运行 **06 半场射击校准**。INIT 时用方向键左 / 右选择半场，将云台轻摆到最右端，再按 START。每个半场有 **3 列 × 6 行，共 18 个格心**，坐标沿用场地左下角为原点、右和前为正、每格 609.6 mm 的约定。

| 06 手柄 1 | 操作 |
| --- | --- |
| 方向键 | 格心模式选列、选行，四角模式用左 / 右切换测点 |
| LB / RB | 目标转速每按一次减 / 加 25 RPM |
| X | 云台刹车或松开，以便手摆方向 |
| RT | 单次试射，云台停稳、飞轮到速后供弹 0.65 秒 |
| A | 确认命中后保存本点最近完成的试射 |
| Y | 标记本点不可达，或解除标记 |
| B | 取消本次试射 |
| Back | 空闲时切换格心与四角精测 |

每次装一颗球，车头朝场地正前方，格心模式将麦轮中心摆在格心。四角精测按屏幕提示，将车身指定角对准地垫同名角，两条车身边沿着接缝，整车放在该地垫内。程序根据车身尺寸换算实际测点坐标，保存供弹开始时的射向与目标 RPM。左右半场分别保存在 Robot Controller，已有旧精测记录会继续参与数据读取与查看。

运行 **07 校准数据查看**，点 INIT 即可浏览记录。方向键左 / 右翻页，**A** 切换已记录和全部点位，**X** 切换密集列表与坐标详情。密集列表每行最多 3 点、每页最多 36 点，适合截图或提取文字。

记录格式为 `行列点:角度×100/RPM`。例如 `110:-4675/3100` 表示第 1 行第 1 列格心、射向 −46.75°、目标 3100 RPM。点号 **0** 为格心，**1–4** 依次为左后、右后、右前、左前，**a–h** 为旧精测。`@右,前` 给出原记录的坐标，单位为格。`?` 表示没有数值，`!` 表示不可达，`~` 表示该位置已停用。

## 离线射击拟合工作台

下载或克隆完整仓库后，双击 [射击拟合工作台/index.html](射击拟合工作台/index.html)，即可离线使用。页面的脚本和样式随仓库提供。初始样本包含 35 条记录，其中 23 个实测点参与拟合、12 个不可达格保留在地图中。

1. 在 **导入测点** 中粘贴 07 的点位文字，或选择 `.txt` / 已保存的 `.json`。点位文字按点号和坐标合并，方案 JSON 加载其中的测点和拟合参数。
2. 点击场地查询任意坐标，点击实测点调整角度、RPM 和权重。调整射向修正阶数、RPM 多项式阶数、正则化与微调量，查看场地向量、纵向剖面、残差和留一验证误差。权重设为 0 时，该点退出拟合。
3. 点击 **保存方案** 导出 `06-fit-project.json`，保存数据、方程、系数和拟合参数。点击 **下载 CSV** 导出 `06-measurements.csv`，查看逐点实测值、预测值和残差。方程区域支持复制。

工作台按场地中线镜像右半场。测点包络之外的外推使用虚线和浅色箭头显示。机器人端的 `Biobuzz角落出发` 从 Robot Controller 上的 06 标定记录生成射击地图，工作台 JSON 用于离线保存和再次导入工作台。详细说明见 [工作台 README](射击拟合工作台/README.md)。

## 比赛操控

### INIT

1. 将机器人摆到目标起点，保持底盘静止，让 Pinpoint 校零。场地左下角为原点，位置以**右、前**为正，每格为 **609.6 mm**。车头面向场地前方为 **0°**，向左为正。
2. 用手柄 1 的方向键录入实际起点，每按一次调整 **0.25 格**。**LB** 将初始航向加 **15°**，**RB** 减 **15°**。默认起点对应车身贴场地左、下两边且车头朝前。
3. 将云台轻轻摆到标定时的最右端，再按 **START**。程序以该位置为本次云台零位。

### START 后

| 手柄 1 | 功能 |
| --- | --- |
| 左摇杆 | 麦轮平移。航向可用时按场地坐标无头驾驶，航向丢失时切换为车头坐标 |
| 右摇杆横向 | 旋转底盘 |
| 方向键左 / 右 | 当前飞轮目标每按一次减 / 加 100 RPM |
| LB | 放下吸取机构并吸取，松开后抬起 |
| RT | 请求供弹和射击，松开后关闭挡弹舵机 |
| RB | 反转清料，同时关闭挡弹舵机。反转优先于射击和吸取 |
| A | 以当前车头方向重设手柄的无头正前方 |
| X | 切换自动瞄准与锁定当前场地方向 |
| Y | 将机器人放回 INIT 输入的起点和朝向后，重新对齐场地坐标 |
| Back（按住） | 查看 Pinpoint、云台和飞轮诊断，含实际加载的 PID |

### Pinpoint 诊断

启动 **goBILDA Pinpoint Example**，INIT 时保持底盘静止，START 后用左摇杆平移、右摇杆横向转向。屏幕显示位置、速度、原始编码器计数、坏读次数和最近异常，异常同时写入 Robot Controller 日志。停稳后按 **A** 将位姿归零并重校 IMU，按 **B** 仅重校 IMU。

主程序按采样间隔检查航向变化与陀螺仪角速度是否一致。I²C 读取失败后，重试间隔从 50 ms 逐步延长到 500 ms，恢复读取后回到正常采样。丢帧时保留上一帧有效位置和航向，航向失效超过 120 ms 后切换为车头模式。需要重新对齐时，将机器人放回 INIT 输入的起点和朝向，再按 **Y**。

### 自动瞄准与射击

- 普通主控默认自动瞄准。场地右坐标小于 3 格时瞄准 **右 2、前 2.5**，到达 3 格时切换到 **右 4、前 2.5**。**X** 切换自动瞄准与锁定方向。
- 禁射区按机器人麦轮中心判断，范围为 **右 2–4 格、前 2.5–3.5 格**，包含边界。区内云台继续瞄准，供弹暂停，挡弹舵机保持关闭。
- 普通主控的飞轮默认目标为 **3100 RPM**，方向键左 / 右每按一次减 / 加 100 RPM，上限由实测功率曲线决定，设为 **0 RPM** 时停转。云台转速连续 **100 ms** 低于约 **7.45°/s**、飞轮转速在目标的 **±10%** 内时，RT 请求可放行。供弹电机缓升到 1.00，吸取舵机刚抬起时先等待 180 ms。已放行的一轮 RT 被禁射区或反转等条件中断后，松开再按以重新检查。

### Biobuzz角落出发

该模式使用 06 保存的格心、四角和旧精测记录，连续计算射向与 RPM。程序选取格心记录较多的半场作基准，同样多时选左侧，再沿场地中线镜像。INIT 显示基准半场，起点录入和驾驶按键沿用普通主控。

移动瞄准按 Pinpoint 实测速度提前最多 **80 ms、120 mm**。空闲飞轮保持 **2500 RPM**，按住 RT 时四轮输出上限为 **50%**。有效定位、可达射击数据、云台对准和飞轮到速持续 **60 ms** 后开始供弹，条件暂时失效时暂停，保持 RT 可在条件恢复后续供。方向键左 / 右微调地图射击转速，每次 100 RPM。

禁射矩形同为右 2–4 格、前 2.5–3.5 格，该模式按实际航向投影车身四角，整车四角都在矩形内时禁射。未测格和标记为不可达的区域也会暂停供弹。

## 代码入口

| 文件或目录 | 用途 |
| --- | --- |
| [BioBuzzTeleOp.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/BioBuzzTeleOp.java) | 比赛主控，协调手柄输入、底盘、定位、瞄准、供弹与状态灯 |
| [RobotConfig.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/RobotConfig.java) | 硬件名称、方向、机械尺寸、里程轮参数和功率限制 |
| [PinpointPose.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/PinpointPose.java) | Pinpoint 读数校验、场地坐标对齐和定位异常处理 |
| [SensorGoBildaPinpointExample.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/SensorGoBildaPinpointExample.java) | 车头模式驾驶、Pinpoint 原始读数与异常日志诊断 |
| [FieldTargets.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/FieldTargets.java) | 场地得分点与禁射区 |
| [Flywheel.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/Flywheel.java) | 飞轮测速、转速 PID 和挡弹舵机 |
| [Turret.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/Turret.java) | 云台角度、回转控制与瞄准状态 |
| [ShotVectorField.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/ShotVectorField.java) / [CalibratedShotMap.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/CalibratedShotMap.java) | 格心、四角与旧精测存储，机器人运行时射击地图 |
| [ShotFieldCalibration.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/calibration/ShotFieldCalibration.java) / [ShotFieldDataView.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/calibration/ShotFieldDataView.java) | 06 打点标定和 07 数据查看 |
| [calibration/](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/calibration) | 自测 OpMode、模型拟合与 PID 参数搜索 |
| [射击拟合工作台/](射击拟合工作台) | 离线数据导入、拟合、可视化和 JSON / CSV 导出 |
| [FtcRobotController/](FtcRobotController) | FIRST SDK 的 Robot Controller 基础工程与示例 |

## 上游项目与许可

项目基于 [FIRST FTC Robot Controller SDK 12.0](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v12.0)。许可和第三方声明见 [LICENSE](LICENSE) 与 [doc/legal/](doc/legal)。
