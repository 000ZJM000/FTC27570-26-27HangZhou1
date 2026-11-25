# Pedro 自动路线

项目使用 PedroPathing 3.0.0，Driver Station 的 Autonomous 提供两套入口。

| 入口 | 当前源文件 | 段数 |
| --- | --- | ---: |
| **BIOBUZZ Pedro 吸两组设三组2T** | [吸两组设三组2T-3.json](吸两组设三组2T-3.json) | 9 |
| **BIOBUZZ Pedro 等待后打预制Park** | [等待后打预制park.json](等待后打预制park.json) | 2 |

其他文件为存档或备用草案：[吸两组设三组2T-2.json](吸两组设三组2T-2.json)、[吸两组设三组2T.json](吸两组设三组2T.json) 各 7 段，[-2.pp.json](-2.pp.json) 为 2 段，当前未作为构建源。

## 修改与打包

`TeamCode/build.gradle` 的 `copyPedroRoute` 将两套路线打包为 `pedro/two-group-route.json`、`pedro/wait-shoot-park-route.json`，同时将 `测点/shot_model.json` 打包为 `biobuzz/shot_model.json`。覆盖源文件后重新构建并安装 APK，切换源文件时同步修改 `routeFile` 或 `waitShootParkRouteFile`。

坐标单位为英寸，起步航向取第 1 段 Heading，以 INIT 显示为准。图中 0° 朝右、90° 朝上、180° 朝左、270° 朝下。支持直线、Bezier 控制点、constant／linear／tangential 航向和段前后等待，按 `sequence` 顺序执行。

## 机构阶段

主路线第 1、4、8 段后射击，第 4 段前默认多等 0.30 秒。第 2 段及等待期间吸取，随后停吸。第 5 段后放下 FR/FL，至少等 0.45 秒。第 6 段持续吸取，段末执行 3 组 0.20 秒后退、1.00 秒前推，输出 0.70，然后从实测位姿接续。第 7 段后收 FR/FL，第 8 段前停吸、射击后停飞轮与云台，第 9 段停靠。

等待后 Park 路线先行驶第 1 段，再按该段 `waitBeforeMs + waitAfterMs` 等待，当前合计 10 秒。先静候 8 秒，最后 2 秒请求射击，完成后停射并执行后续 Park 路段。实际达速与供弹等待受 `AUTO_VOLLEY_*` 截止时间约束。

改变主路线的段数和阶段顺序时，同步调整 [BioBuzzPedroAuto.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/opmode/BioBuzzPedroAuto.java) 的机构流程。两套入口共用定位、射界和接续逻辑，参数在 Dashboard 的 `RobotConfig` 面板调整。详细操作见 [项目 README](../README.md) 和 [上车指南](../BIOBUZZ-上车指南.md)。
