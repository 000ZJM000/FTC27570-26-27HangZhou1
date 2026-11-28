# 标定、测点与模型存档

| 文件 | 记录数 | 用途 |
| --- | ---: | --- |
| [shot_records.json](shot_records.json) | 18 | 当前实测 records，可导入工作台 |
| [shot_model.json](shot_model.json) | 46 | 当前模型及 points，随 APK 打包 |
| [shot_model-V1.json](shot_model-V1.json) / [shot_records-V1.json](shot_records-V1.json) | 各 35 | V1 模型与记录存档 |
| [shot_model-V2.json](shot_model-V2.json) | 46 | V2 模型存档 |
| [shot_model-V3.json](shot_model-V3.json) / [shot_records-V3.json](shot_records-V3.json) | 各 46 | V3 模型与记录存档 |
| [shot_model-V4.json](shot_model-V4.json) / [shot_records-V4.json](shot_records-V4.json) | 各 46 | V4 模型与记录存档 |
| [shot_model-V5.json](shot_model-V5.json) | 46 | V5 模型存档 |
| [calibration.json](calibration.json) | — | 实车硬件标定快照，包含方向、行程、PID 与本机测点等保存值 |

模型和 records 使用 JSON 格式版本 2，文件名中的 V1–V5 是存档批次。表中数量是 `points` 总数，实际参与拟合的点由状态及权重决定。`calibration.json` 使用版本 1 的 `values` 结构，可用于本机标定备份与恢复。

在 [射击拟合工作台](../射击拟合工作台/index.html) 导入 records 读取测点，导入 model 恢复方案，保存后得到新的 `shot_model.json`。新版拟合支持整体几何零位、有界方向 RPM 修正、Huber 抗噪和自动寻优。少点迁移结果中包含估算点，可在 06 中补测验证。

车端优先读取 Hub `/sdcard/FIRST/biobuzz/shot_model.json` 的有效模型，再读取 APK 内置模型，之后才尝试从 06 测点拟合。仓库当前模型随构建打包到 `assets/biobuzz/shot_model.json`，替换内置版本后重新构建并安装。INIT 显示实际模型来源。

06 从本机 `calibration.json` 导出 records。模型文件可携带比最新 records 更多的历史或迁移点，因此当前两者分别为 46 与 18 条。具体操作见 [项目 README](../README.md) 和 [工作台说明](../射击拟合工作台/README.md)。
