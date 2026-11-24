# 射击测点与模型存档

| 文件 | 记录数 | 用途 |
| --- | ---: | --- |
| [shot_records.json](shot_records.json) | 46 | 当前实测记录包，可导入离线工作台 |
| [shot_model.json](shot_model.json) | 46 | 当前拟合方案与模型，随 APK 打包，也可导入工作台或回传 Hub |
| [shot_records-V1.json](shot_records-V1.json) | 35 | V1 实测记录存档 |
| [shot_model-V1.json](shot_model-V1.json) | 35 | V1 拟合方案与模型存档 |

表中数量为 `points` 的总记录数，参与拟合的数量由点位状态和权重决定。四份文件的 JSON 格式版本均为 2，文件名中的 V1 表示存档批次。

在 [射击拟合工作台](../射击拟合工作台/index.html) 中导入 records 可读取测点，导入 model 可恢复方案与参数。保存方案后得到新的 `shot_model.json`。

将模型以 `shot_model.json` 为文件名复制到 Control Hub 的 `/sdcard/FIRST/biobuzz/`，下一次 INIT 手动或 Pedro 自动时优先加载本机有效模型。本机模型缺失或无效时，先用 APK 内置模型，再尝试 06 测点拟合。Gradle 自动将本目录的 `shot_model.json` 打包到 `assets/biobuzz/shot_model.json`，修改内置版本后重新构建并安装 APK。

车上的 06 从本机 `calibration.json` 自动导出 `shot_records.json`。records 用于向电脑交换测点，恢复本机校准使用 `calibration.json`。完整操作见 [项目 README](../README.md) 的射击标定与模型交换章节。
