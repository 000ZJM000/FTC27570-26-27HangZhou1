# Pedro 自动路线

当前自动入口为 **BIOBUZZ Pedro 吸两组设三组2T**，使用 PedroPathing 3.0.0。

| 文件 | 用途 |
| --- | --- |
| [吸两组设三组2T-3.json](吸两组设三组2T-3.json) | 当前构建使用的 8 段路线，与自动机构流程对应 |
| [吸两组设三组2T-2.json](吸两组设三组2T-2.json) | 7 段路线存档 |
| [吸两组设三组2T.json](吸两组设三组2T.json) | 7 段路线存档 |

在路径编辑器修改后，覆盖当前文件，再构建并安装 `TeamCode`。`TeamCode/build.gradle` 的 `copyPedroRoute` 自动将源文件打包成 `pedro/two-group-route.json`，自动和手动 INIT 均使用 APK 中的路线起点。切换源文件需同时修改 Gradle 的 `routeFile`，现有自动程序要求 8 段。

坐标单位为英寸，起始航向取第 1 段的 Heading。图中 0° 朝右、90° 朝上、180° 朝左、270° 朝下。当前第 1 段使用 linear，起始与结束航向均为 180°。程序支持直线、Bezier 控制点、constant／linear／tangential 航向及段前后等待，按 `sequence` 顺序执行。

第 1、3、7 段后尝试射击，第 2、5–7 段吸取，第 4 段到位后放下 FR/FL，第 8 段停车并收回 FR/FL。修改段数或阶段顺序时，同步调整 `BioBuzzPedroAuto.java` 的机构流程。摆位、供弹计时与超时衔接见 [项目 README](../README.md) 和 [上车指南](../BIOBUZZ-上车指南.md)。
