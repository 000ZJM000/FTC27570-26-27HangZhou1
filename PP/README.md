# Pedro 自动路线

项目使用 PedroPathing 3.0.0，Autonomous 共有四套入口。

| Driver Station 名称 | 源文件 | 段数 | APK 路径 |
| --- | --- | ---: | --- |
| BIOBUZZ Pedro 吸两组设三组2T | [吸两组设三组2T-3.json](吸两组设三组2T-3.json) | 9 | `pedro/two-group-route.json` |
| BIOBUZZ Pedro 等待后打预制Park | [等待后打预制park.json](等待后打预制park.json) | 2 | `pedro/wait-shoot-park-route.json` |
| BIOBUZZ Pedro 直接停近 | [直接停近.json](直接停近.json) | 1 | `pedro/direct-park-near-route.json` |
| BIOBUZZ Pedro 直接停远 | [直接停远.json](直接停远.json) | 1 | `pedro/direct-park-far-route.json` |

[吸两组设三组2T-2.json](吸两组设三组2T-2.json) 和 [吸两组设三组2T.json](吸两组设三组2T.json) 是各 7 段的存档，当前未打包。旧 `-2.pp.json` 已移除。

## 修改与构建

`TeamCode/build.gradle` 的 `copyPedroRoute` 将四条路线与 `测点/shot_model.json` 一起打包。覆盖源文件后重新构建、安装 APK，切换构建源时同步修改对应 `routeFile` 属性。JSON 坐标单位为英寸，起始航向取第 1 段 Heading，以 INIT 显示为准。图中 0° 朝右、90° 朝上、180° 朝左、270° 朝下。

支持直线、Bezier、constant／linear／tangential 航向和段前后等待，按 `sequence` 顺序执行。主路线的段数和顺序与机构动作绑定，修改时同步调整 [BioBuzzPedroAuto.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/opmode/BioBuzzPedroAuto.java)。

## 模式行为

主路线第 1、4、8 段后射击。第 2 段吸取，段末默认以 0.60 输出前推 0.80 秒，再由实测位姿衔接。第 5 段后放下 FR/FL 并至少等 0.45 秒。第 6 段吸取及往复前推，第 7 段后收 FR/FL，第 8 段前停吸、射击后停射，第 9 段停靠。参数使用 Dashboard 的 `AUTO_*` 与 `PEDRO_*` 字段。

等待后 Park 当前第 1 段等待合计 10 秒，到位后先静候 8 秒，最后 2 秒请求射击，再执行 Park。实际供弹受达速条件与供弹截止时间控制。

直接停近 / 停远各执行一条路径，不启用自动射击，START 时 FR/FL 保留 INIT 位置。两套新入口分别为 [BioBuzzDirectParkNearAuto.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/opmode/BioBuzzDirectParkNearAuto.java) 和 [BioBuzzDirectParkFarAuto.java](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/opmode/BioBuzzDirectParkFarAuto.java)。

各模式共用 Pinpoint 定位、超时处理和状态接续。摆位与操作见 [项目 README](../README.md) 和 [上车指南](../BIOBUZZ-上车指南.md)。
