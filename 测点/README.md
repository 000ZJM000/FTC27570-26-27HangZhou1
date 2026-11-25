# 射击测点与模型存档

| 文件 | points 总数 | 用途 |
| --- | ---: | --- |
| [shot_model.json](shot_model.json) | 46 | 当前模型，随 APK 打包，也可导入工作台或回传 Hub |
| [shot_records.json](shot_records.json) | 46 | 实测记录包，可导入工作台 |
| [shot_model-V1.json](shot_model-V1.json) / [shot_records-V1.json](shot_records-V1.json) | 各 35 | V1 模型与记录存档 |
| [shot_model-V2.json](shot_model-V2.json) | 46 | V2 模型存档 |
| [shot_model-V3.json](shot_model-V3.json) / [shot_records-V3.json](shot_records-V3.json) | 各 46 | V3 模型与记录存档 |

文件均使用 JSON 格式版本 2，文件名中的 V1/V2/V3 表示存档批次。`points` 数量包含停用或不可达点，实际参与拟合的数量由状态和权重决定。

在 [射击拟合工作台](../射击拟合工作台/index.html) 导入 records 读取测点，导入 model 恢复方案。保存方案得到新的 `shot_model.json`，复制到 Hub 的 `/sdcard/FIRST/biobuzz/` 后重新 INIT，即可优先加载本机有效模型。

本机模型缺失或无效时，先使用 APK 内置模型，再从 06 测点拟合。Gradle 将本目录 `shot_model.json` 打包为 `assets/biobuzz/shot_model.json`，替换内置版本后重新构建并安装 APK。新补的 06 点不会覆盖优先级更高的有效模型，INIT 显示实际来源。

车端支持按半场独立模型和固定偏置。06 拟合时，两半场各至少 6 个有效点可独立拟合，少量对侧点用于估计对称角偏置；Dashboard 的 `SHOT_LEFT/RIGHT_*` 字段可分别调整固定角度和 RPM 偏置。

06 会从本机 `calibration.json` 自动导出 `shot_records.json`。records 用于交换测点，本机硬件校准的备份与恢复使用 `calibration.json`。完整操作见 [项目 README](../README.md)。
