# FTC27570-26-27HangZhou1

FTC 27570 在 2026–2027 BIOBUZZ 赛季使用的机器人控制程序。项目基于 FIRST FTC SDK 12.0，主控程序集成麦克纳姆底盘、goBILDA Pinpoint 双里程轮定位、云台自动瞄准、双飞轮与供弹联锁，以及 KIRIN 车尾状态灯。云台和飞轮按标称 12 V 进行电压补偿，射击数据支持从 Hub 导出、离线拟合和模型回传。

- **比赛主控** · Driver Station → TeleOp → **BIOBUZZ 360 云台 + 无头**
- **标定地图跑打** · Driver Station → TeleOp → **BIOBUZZ → Biobuzz角落出发**
- **自动路线** · Driver Station → Autonomous → **BIOBUZZ Pedro 吸两组设三组2T**，执行 8 段路径与机构动作
- **硬件自测** · Driver Station → TeleOp → **BIOBUZZ 自测**，首次运行 **02 云台全套 → 04 飞轮校准**
- **射击标定与查看** · 完成硬件自测后运行 **06 半场射击校准·数据**，START 后按 **L3** 切换数据查看
- **转向标定** · Driver Station → TeleOp → **BIOBUZZ 自测 → 07 转向摇杆线性校准**
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

`TeamCode/build.gradle` 已配置 WAV、MP3、OGG 音频资源无压缩打包，并集成 FTC Dashboard 0.6.0 与 PedroPathing `com.pedropathing:revhub:3.0.0`。构建时自动将指定 PP 路线打包为 assets，首次同步需能访问依赖仓库。

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
| 任务舵机（成对使用） | `FR`、`FL` | 按实车接线设置 Hub 与端口，代码未固定端口 |
| goBILDA Pinpoint | `pinpoint` | Expansion Hub 2 I²C Bus 1，接两个里程轮 |
| KIRIN V3 尾灯（可选） | `LED` | 接线所用 I²C 总线，设备类型选 **STM32 Smart LED Module V3** |

飞轮方向以 04 保存的结果为准。Pinpoint 使用 FTC SDK 的 goBILDA 4-Bar 里程轮预设，机械尺寸与安装方向集中在 `RobotConfig.java`。FR/FL 初始化和收回位置为 **0.94 / 0.06**，放下为 **0.00 / 1.00**，需两只均配置才执行切换。

## 第一次上车

清空仓内和供弹通道，给底盘和云台留出运动空间。在 Driver Station 启动各自测后，按 **A** 确认，按 **B** 跳过提示或停止当前动作，按 **STOP** 退出 OpMode。

| 用途 | Driver Station 名称 | 自测内容 |
| --- | --- | --- |
| 云台全套 | **02 云台全套 12V补偿·联动·PID** | 行程与方向、动力学、六点双向静／动摩擦、底盘联动、PID 与回转验收 |
| 飞轮校准 | **04 飞轮校准 12V前馈·三球回速** | 转向与每转计数、七档空载电压前馈拟合，可选三球实弹回速 |
| 单项复测 | **03 云台专项 12V联动补偿** | 使用现有行程与动力学，单独重测底盘联动 |
| 快检或重调 | **05 云台专项 12V PID·回转** | 使用现有行程与摩擦，单独运行 30 秒快检或完整 PID 拟合 |
| 转向手感 | **07 转向摇杆线性校准** | 自动测左右起转死区、七档旋转速度，生成转向映射并复测 |

首次校准运行 **02 → 04**。02 可在已有有效标定基础上，分阶段选择重测或沿用动力学、摩擦表、底盘联动和云台 PID。执行联动与回转前按提示让轮子落地、腾出旋转空间。04 按提示识别飞轮方向、测每转计数并扫描七档 3–12 V 前馈，之后可进行三球回速测试。

02 从静止缓升电压测起转阈值，再用独立匀速行程测动摩擦。定位无进展时沿目标方向缓升电压重测起转，受行程和安全电压限制。未检测到起转的位置按缺测处理，可换偏置角补测。结果分别显示静摩擦、动摩擦的左右有效点数。覆盖不足时沿用旧表，反馈正常时继续后续联动与 PID 校准。

飞轮调速使用 12 V 前馈、固定轻量 PI 和带滞回的 Bang-Bang 补电，默认 P/I/D 为 **1.6 / 0.30 / 0**。冷启动软起动约 **400 ms**，工作段掉速超过 **120 RPM** 时满功率补电，偏差回到 **50 RPM** 内结束补电。超速超过 **180 RPM** 时空转滑行。INIT 若提示使用旧占空比曲线，可运行 04 重测 12 V 前馈。

飞轮开环扫描的低压起转阶段单独处理失转计时。编码器长时间无运动或读取异常时，`BioBuzzFlywheel` 日志记录故障，失转记录包含实际输出电压、电池电压和目标 RPM。

标定存于 Control Hub 的 `/sdcard/FIRST/biobuzz/calibration.json`。程序首次读取时会迁移旧标定。换 Hub 时可备份和恢复此文件。行程、方向或动力学改变后，受影响的数据保留并标为过期，主程序 INIT 黄字提示重测。

校准屏显示实测电池电压与 12.00 V 标称值。主程序 INIT 会显示云台和飞轮实际加载的 P/I/D 与来源。运行中按住 **Back** 可以查看实际输出、电压与转速偏差。

## 转向摇杆线性校准

运行 **BIOBUZZ 自测 → 07 转向摇杆线性校准**。轮子落地，留出底盘旋转空间，初始化 Pinpoint 时保持静止。START 后按提示选择：首次按 **A** 开始测量，已有有效函数时按 **A** 重测、按 **B** 直接复测现有函数。

程序先测左右起转死区，再各扫七档输出，按左右方向分别拟合三次逆映射。有效拟合会自动保存到 `calibration.json` 的 `drive_turn_linearity_v1`，随后以 20%、40%、60%、80%、100% 杆量双向复测，显示校准函数、RMSE、R² 与峰值角速度。扫描中止或拟合失败时保留原参数。

复测后按 **A** 可进入最长 45 秒的实时试驾，左摇杆平移、右摇杆横向转向，**X** 结束、**B** 停止当前动作。普通主控与角落模式在下一次 INIT 加载该函数，并显示加载状态。转向按 12 V 换算输出，平移时减少起转补偿，未保存有效函数时沿用摇杆直通。平移叠加、功率上限和地面负载会影响实际角速度，以复测结果为准。

## Hub 标定文件与模型交换

默认数据目录为 `/sdcard/FIRST/biobuzz/`，文件用途如下。

| 文件 | 来源与用途 |
| --- | --- |
| `calibration.json` | 保存硬件标定、PID、转向映射、射击测点和过期标记，可用于换 Hub 时备份与恢复 |
| `shot_records.json` | 06 自动导出的实测记录，包含坐标、射向、RPM、机械尺寸、电池电压和 ST 位置，数据查看模式按 Y 可重导 |
| `shot_model.json` | 工作台导出的参数模型，复制回同目录后，在下一次 INIT 角落模式时加载 |

`CalibrationStore` 从旧版应用标定迁移到 JSON，写入使用临时文件和原子替换。共享目录不可写时使用应用内部的 `biobuzz` 目录。仓库中的 [calibration.json](TeamCode/src/main/assets/biobuzz/calibration.json) 提供空结构模板，实车数值由运行中的校准程序保存。

## 射击数据标定与查看

完成 02 的云台行程与正前方向、04 的飞轮 RPM 标定后，运行 **06 半场射击校准·数据**。INIT 时用方向键左 / 右选择半场，将云台轻摆到最右端，再按 START。每个半场有 **3 列 × 6 行，共 18 个格心**，坐标沿用场地左下角为原点、右和前为正、每格 609.6 mm 的约定。

| 06 手柄 1 | 操作 |
| --- | --- |
| 方向键 | 格心模式选列、选行，四角模式用左 / 右切换测点 |
| LB / RB | 目标转速每按一次减 / 加 25 RPM |
| X | 云台刹车或松开，以便手摆方向 |
| R3 | 按当前模型预瞄选中点，B 取消 |
| L3 | 空闲时切换试射与数据查看 |
| RT | 单次试射，云台停稳、飞轮到速后供弹 0.65 秒 |
| A | 确认命中后保存本点最近完成的试射 |
| Y | 标记本点不可达，或解除标记 |
| B | 取消本次试射 |
| Back | 空闲时切换格心与四角精测 |

每次装一颗球，车头朝场地正前方，格心模式将麦轮中心摆在格心。四角精测按屏幕提示，将车身指定角对准地垫同名角，两条车身边沿着接缝，整车放在该地垫内。程序根据车身尺寸换算实际测点坐标，保存供弹开始时的射向、目标 RPM、电池电压和 ST 舵机位置。有新版四角记录的格子会自动停用旧精测。

在 **06 半场射击校准·数据** 中，START 后空闲时按 **L3** 进入数据查看，浏览两个半场的格心、四角与旧精测。方向键左 / 右翻页，**A** 切换已记录和全部点位，**X** 切换密集列表与坐标详情，**Y** 重新导出 `shot_records.json`，**L3** 或 **B** 返回试射。密集模式每行最多 3 个点、每页最多 10 行，详情每页 3 个点。进入 START、保存测点和变更不可达标记时也会自动导出记录。

文件管理快捷键在 INIT 或 START 后的空闲状态可用。

| 06 组合键 | 作用 |
| --- | --- |
| START + X | 删除 `shot_records.json` 和 `shot_model.json`，保留 `calibration.json` 内的测点，记录可再次导出 |
| START + Y | 清空两个半场的旧版 v1 精测点并更新记录 JSON |
| START + Back，3 秒内松开再按一次 | 清空全部 06 测点及射击元数据，并删除 records / model 两个 JSON |

清空全部 06 数据会保留云台、飞轮、转向等硬件标定。删除模型后，下次 INIT 角落模式将尝试从保留的 06 测点拟合。

记录格式为 `行列点:角度×100/RPM`。例如 `110:-4675/3100` 表示第 1 行第 1 列格心、射向 −46.75°、目标 3100 RPM。点号 **0** 为格心，**1–4** 依次为左后、右后、右前、左前，**a–h** 为旧精测。`@右,前` 给出原记录的坐标，单位为格。`?` 表示没有数值，`!` 表示不可达，`~` 表示该位置已停用。

## 离线射击拟合工作台

下载或克隆完整仓库后，双击 [射击拟合工作台/index.html](射击拟合工作台/index.html)，即可离线使用。页面的脚本和样式随仓库提供。初始样本包含 35 条记录，其中 22 个新版实测点参与拟合。

1. 将 `/sdcard/FIRST/biobuzz/shot_records.json` 从 Hub 复制到电脑，在 **导入测点** 中选取该文件。
2. 点击场地查询任意坐标，点击实测点调整角度、RPM 和权重。射向由等效得分点与标准化多项式残差拟合，RPM 按到得分点的距离拟合。调整阶数、正则化和微调量，查看场地向量、纵向剖面、残差及加权留一验证误差。权重设为 0 时，该点退出拟合。
3. 点击 **保存方案** 导出 `shot_model.json`，复制到 Hub 的同一目录。下一次 INIT 角落模式时会加载该模型。也可下载 CSV 查看逐点残差。

工作台按场地中线镜像右半场，机械角度微调在镜像后叠加。导出的版本 2 模型携带 `geometry`、`model`、测点和拟合参数，计算使用导出机械尺寸对应的云台转轴坐标。测点包络之外的外推使用虚线和浅色箭头显示。角落模式找不到可用模型文件时，按工作台默认参数从 06 测点拟合。详细说明见 [工作台 README](射击拟合工作台/README.md)。

### 固定俯仰角变更后的少点迁移

调整发射器固定俯仰角后，可重测少量锚点，用新旧偏差估算其余格心与精测记录。常用锚点是覆盖近远距离的 6 个格心 `110`–`610`。

1. 加载已有的完整测点或方案，在 **新角度少点校准** 中点击 **将当前表设为基准**。
2. 粘贴新锚点的 06 数据查看文字或读取 JSON/TXT，点击 **填入锚点表**，核对自动勾选的点、射向和 RPM。也可用 **提取格心锚点** 快速选择格心。
3. 选择迁移算法、射向处理和 RPM 径向阶数，查看各点的旧值、新值、迁移量及计算方式。默认使用格心锚定与径向梯度，详细策略见 [工作台 README](射击拟合工作台/README.md)。
4. 点击 **应用校准到全部精测点** 更新主表，再保存 `shot_model.json` 回传 Hub。**还原到 24 点基准** 可将主表恢复为当前保存的基准，实际迁移点数显示在预览统计中。

在普通导入窗口勾选 **作为新角度少点锚点**，会按当前迁移设置直接应用导入数据。保存的方案会同时包含 `baselinePoints`，便于再次导入后继续使用原基准。迁移后的估算点可通过 06 补测试射验证。

## Pedro 自动路线

Driver Station → Autonomous → **BIOBUZZ Pedro 吸两组设三组2T**。使用 PedroPathing 3.0.0 的 Follower、Foresight 和麦轮驱动，定位由现有 Pinpoint 提供，自动与角落主控共用射界查询和供弹判定。

当前打包路线为 [PP/吸两组设三组2T-3.json](PP/吸两组设三组2T-3.json)，共 **8 段**。Gradle 的 `copyPedroRoute` 将它生成为 APK 内的 `pedro/two-group-route.json`。覆盖源文件后重新构建并安装 APK，INIT 时加载更新。其他两份 PP 文件为 7 段存档，现有机构流程要求 8 段。格式与更新方式见 [PP/README.md](PP/README.md)。

1. 按 INIT 显示的 PP 起点 X、Y 和车头角摆车，单位为英寸。起始航向取第 1 段的 Heading，场地图上 0° 朝右、90° 朝上、180° 朝左、270° 朝下。
2. 保持底盘静止，让自动初始化 Pinpoint，将云台轻摆到最右端。INIT 会抬起 SL/SR、关闭 SB、设置 ST，并将 FR/FL 置于 `0.94 / 0.06`。
3. 按 START 采云台零位并执行路线，屏幕显示路径、定位、射界来源和射击状态。

| 路径阶段 | 机构动作 |
| --- | --- |
| 第 1、3 段后等待 | 原地自瞄，按供弹判定尝试连续射击 |
| 第 2 段及其段后等待 | 放下 SL/SR 吸取，等待结束后停止并抬起 |
| 第 4 段到位 | 放下 FR/FL，至少等待 180 ms |
| 第 5–7 段 | 持续吸取，第 7 段后抬 SL/SR，再原地自瞄射击 |
| 第 8 段 | 停止飞轮与云台输出，行驶过半收回 FR/FL |

每轮射击累计实际供弹 **2 秒** 后收闸，并至少保留 PP 原定等待。等待上限为原定等待与 2 秒中的较大值再加 2 秒。射界不可用时保朝向并用默认转速，供弹仍经过机构与禁射区判定。

未到位且平移低于 **0.5 in/s**、转向低于 **4°/s** 连续 **4.5 秒** 时，刹车后跳过该段，再从实测位姿接下一段。第 1、3、7 段超时后仍尝试射击，第 4 段需到位才放下 FR/FL。Pinpoint 短暂失读时刹车等待，持续失效约 **250 ms** 时停止自动。路线与控制参数仍需实车确认。

自动结束或中途 STOP 后，保持 Pinpoint 通电再切手动。两种手动以同一 PP 起点解释传感器累计读数，INIT 的「接续位姿」用于核对当前位置。

## 比赛操控

### INIT

1. 两种手动模式沿用 Pinpoint 已累计的位移与航向，按 APK 中的 PP 起点转换为场地坐标。INIT 显示「PP 起点」和「接续位姿」，切换自动与手动时保持传感器通电。
2. 核对「接续位姿」。需要重新对齐时，将机器人放回原手动角并朝原手动正前方，START 后按 **Y**。场地位置仍以右、前为正，每格 **609.6 mm**，正前为 0°、向左为正。
3. 将云台轻轻摆到标定时的最右端，再按 **START** 采本次云台零位。手动 INIT 使用 Pinpoint 现有读数，自动 INIT 执行传感器校零。

### START 后

| 手柄 1 | 功能 |
| --- | --- |
| 左摇杆 | 麦轮平移。航向可用时按场地坐标无头驾驶，航向丢失时切换为车头坐标 |
| 右摇杆横向 | 旋转底盘，加载有效 07 校准后使用左右转向映射与电压补偿 |
| 方向键左 / 右 | 当前飞轮目标每按一次减 / 加 100 RPM |
| 方向键上 | 同步切换 FR/FL 的收回与放下位置 |
| Back + 方向键上 / 方向键下 | 云台零位每按一次加 / 减 0.5° |
| R3（未按 Back）/ L3 | 切换转速锁 / 清零云台微调 |
| Back + R3 | 切换云台解耦 |
| LB | 放下吸取机构并吸取，松开后抬起 |
| RT | 请求供弹和射击，松开后关闭挡弹舵机 |
| RB | 反转清料，同时关闭挡弹舵机。反转优先于射击和吸取 |
| A | 以当前车头方向重设手柄的无头正前方 |
| X | 切换自动瞄准与锁定当前场地方向 |
| Y | 将机器人放回原手动角并朝原手动正前方后，重新对齐场地坐标 |
| Back（按住） | 查看 Pinpoint、云台和飞轮诊断，含实际加载的 PID |

手柄 2 的 R3 也可切换云台解耦。解耦时云台电机停止输出，RT 跳过角度等待。云台未配置、未标定、反馈不可用，或场地航向及最近位置不可用时，也会跳过角度等待。供弹仍受禁射、目标转速、飞轮与供弹机构状态限制。

尾灯绿色表示已对准，青色表示已对准且开启转速锁，黄色表示云台解耦。

### Pinpoint 诊断

启动 **goBILDA Pinpoint Example**，INIT 时保持底盘静止，START 后用左摇杆平移、右摇杆横向转向。屏幕显示位置、速度、原始编码器计数、坏读次数和最近异常，异常同时写入 Robot Controller 日志。停稳后按 **A** 将位姿归零并重校 IMU，按 **B** 仅重校 IMU。

主程序按采样间隔检查航向变化与陀螺仪角速度是否一致。I²C 读取失败后，重试间隔从 50 ms 逐步延长到 500 ms，恢复读取后回到正常采样。丢帧时保留上一帧有效位置和航向，航向失效超过 120 ms 后切换为车头模式。需要重新对齐时，将机器人放回原手动角并朝原手动正前方，再按 **Y**。

### 自动瞄准与射击

- 普通主控默认自动瞄准。场地右坐标小于 3 格时瞄准 **右 2、前 2.5**，到达 3 格时切换到 **右 4、前 2.5**。**X** 切换自动瞄准与锁定方向。
- 禁射区按机器人麦轮中心判断，范围为 **右 2–4 格、前 2.5–3.5 格**，包含边界。区内云台继续瞄准，供弹暂停，挡弹舵机保持关闭。
- 普通主控的飞轮默认目标为 **3100 RPM**，目标设为 **0 RPM** 时停转。开机为高速直射。云台达到 PID 校准给出的角度阈值，或连续 **100 ms** 低于约 **7.45°/s**，即可通过本轮 RT 的角度判定。供弹直接升到 1.00，吸取舵机刚抬起时先等待 180 ms。已放行的一轮 RT 被禁射区、反转或飞轮／供弹故障中断后，松开再按以重新检查。
- 开启转速锁时，在目标的 **−8% 至 +12%** 入闸，连续供弹期间低于目标 **82%** 暂停。电池低于 **10.2 V** 时，底盘输出上限降低到 **80%**，Driver Station 顶部显示低压提示。

### Biobuzz角落出发

该模式把两半场测点换算到同一个基准半场，拟合等效得分点、射向残差及距离与 RPM 的关系。INIT 显示模型来源，定位接续和驾驶按键沿用普通主控。

当前 `RobotConfig.ENABLE_MOVING_LEAD = false`，角落模式按云台转轴的当前位置查询射击地图。启用该开关后，按机构延迟、飞行时间估计与刹停模型计算平移提前量，上限为 **450 mm**。高速直射时飞轮持续跟随当前目标，缺少可用的 06 数据时用默认转速加微调。禁射区内，或开启转速锁且松开 RT 时，飞轮保持至多 **2500 RPM**。正常电压下底盘输出上限为 **100%**，按住 **RT** 跑打时同样适用，低压时仍受 80% 上限约束。

禁射矩形同为右 2–4 格、前 2.5–3.5 格，该模式按实际航向投影车身四角，定位有效且整车四角都在矩形内时禁射。

仓库的 [测点/](测点) 提供当前 46 条记录的 `shot_records.json` 与 `shot_model.json`，另有 35 条记录的 V1 存档。使用方式见 [测点/README.md](测点/README.md)，可导入工作台或按文件用途复制到 Hub。

## 代码入口

| 文件或目录 | 用途 |
| --- | --- |
| [BioBuzzTeleOp.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/BioBuzzTeleOp.java) | 比赛主控、Pinpoint 位姿接续、手柄输入与机构协调 |
| [BioBuzzPedroAuto.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/BioBuzzPedroAuto.java) / [PedroRoute.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/PedroRoute.java) | 自动路径执行、JSON 路线解析与超时衔接 |
| [PedroPinpointLocalizer.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/PedroPinpointLocalizer.java) | Pinpoint 与 Pedro 的毫米／英寸坐标转换 |
| [PedroAutoMechanisms.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/PedroAutoMechanisms.java) / [AutoVolley.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/AutoVolley.java) | 自动机构阶段、射击等待与实际供弹计时 |
| [CornerAim.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/CornerAim.java) / [ShotSequence.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/ShotSequence.java) | 自动与手动共享射界查询、禁射区和供弹判定 |
| [TaskServos.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/TaskServos.java) | FR/FL 成对切换与停机收回 |
| [RobotConfig.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/RobotConfig.java) | 硬件名称、方向、机械尺寸、里程轮参数和功率限制 |
| [BatteryVoltage.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/BatteryVoltage.java) | 电池采样、12 V 标称电压与占空比换算、低压状态 |
| [ControlLoop.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/ControlLoop.java) / [HardwareStop.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/HardwareStop.java) | 15 ms 控制周期、逐帧 Hub 缓存与异常停机处理 |
| [CalibrationStore.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/CalibrationStore.java) / [ShotRecords.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/ShotRecords.java) | JSON 标定持久化、旧数据迁移和实测记录导出 |
| [ShotMotion.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/ShotMotion.java) / [LeastSquares.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/LeastSquares.java) | 移动与刹停预测、模型最小二乘求解 |
| [PinpointPose.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/PinpointPose.java) | Pinpoint 读数校验、场地坐标对齐和定位异常处理 |
| [SensorGoBildaPinpointExample.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/SensorGoBildaPinpointExample.java) | 车头模式驾驶、Pinpoint 原始读数与异常日志诊断 |
| [FieldTargets.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/FieldTargets.java) | 场地得分点与禁射区 |
| [Flywheel.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/Flywheel.java) | 飞轮测速、12 V 前馈、轻 PI、Bang-Bang 补电和挡弹控制 |
| [Turret.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/Turret.java) | 云台角度、回转控制与瞄准状态 |
| [ShotVectorField.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/ShotVectorField.java) / [CalibratedShotMap.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/CalibratedShotMap.java) | 格心、四角与旧精测存储，机器人运行时射击地图 |
| [ShotFieldCalibration.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/calibration/ShotFieldCalibration.java) | 06 打点标定、数据查看与射击文件管理 |
| [TurnCalibration.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/calibration/TurnCalibration.java) / [TurnLinearity.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/TurnLinearity.java) | 07 自动转向标定、分方向拟合与主控转向映射 |
| [TurretPidTuning.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/calibration/TurretPidTuning.java) | 云台 10–16 个有效轮的 PID 搜索、留出复测与回转验收 |
| [calibration/](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/calibration) | 自测 OpMode、模型拟合与 PID 参数搜索 |
| [射击拟合工作台/](射击拟合工作台) | 离线数据导入、拟合、可视化和 JSON / CSV 导出 |
| [PP/](PP) / [测点/](测点) | 自动路线源文件、实测记录与射界模型存档 |
| [FtcRobotController/](FtcRobotController) | FIRST SDK 的 Robot Controller 基础工程与示例 |

## 上游项目与许可

项目基于 [FIRST FTC Robot Controller SDK 12.0](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v12.0)。许可和第三方声明见 [LICENSE](LICENSE) 与 [doc/legal/](doc/legal)。
