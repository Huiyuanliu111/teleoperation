# Selector 对照实验部署

Maze 和 Threading 分别比较 **no selector（固定执行 10 步）** 与 **selector（动态执行 4–10 步）**。
每个条件评估 20 次，即每个任务 40 次，两个任务共 80 次。每个任务使用同一套 20 个起始条件，
第 i 次在两种方法下尽量复现相同的物体、起始位姿和目标位置。

| 任务 | 条件 | 执行步数 | 归一化节点 | 评估次数 |
| --- | --- | --- | --- | --- |
| Maze | no selector | 固定 10 | 不使用 | 20 |
| Maze | selector | 4–10 | 30% | 20 |
| Threading | no selector | 固定 10 | 不使用 | 20 |
| Threading | selector | 4–10 | 50% | 20 |

两组均生成完整的 10 步动作预测，再执行指定长度的前缀。同一任务使用相同的 ARP checkpoint、
`model` 权重、7.5 Hz 控制频率、相机标定及动作限制，仅改变执行长度是否由 selector 决定。
不启用 endpoint schedule 或额外的低置信度回退。

## 本次模型版本

当前两个任务均使用 `h=4, H=10`：`models/maze_h4` 与 `models/threading_h4`。
模型继续预测精细区概率 p，执行长度为 `floor(4*p + 10*(1-p) + 0.5)`。
概率监督与 h 无关，因此复用已训练权重，不需要重训；原始 h3 训练记录保留。
Maze 使用训练最佳 epoch 7；Threading 使用训练完成后的最佳 epoch 21，权重哈希见
[h4 模型清单](../data/analysis/selector_eval_20260914_142324/manifest_h4.json)。
Threading 的原 epoch 7 评估快照已缺失，本次 h4 使用的权重不应视为该历史快照。

评估目录：`data/analysis/selector_eval_20260914_142324`。
有 selector 的 h4 实验使用独立的 `_selector_h4.csv` / `.jsonl`，用于区分先前 h3 结果。
无 selector 仍固定 10 步，沿用原文件名。

## 环境与公共变量

在交互式终端中运行以下准备命令。部署使用 `pushbox` 环境：本机该环境具备 Pinocchio 和
RealSense 依赖，训练使用的 `arp` 环境缺少这两个部署依赖。

```bash
cd /home/huiyuan/teleoperation
source /home/huiyuan/miniconda3/etc/profile.d/conda.sh
conda activate pushbox

EVAL_ROOT="/home/huiyuan/teleoperation/data/analysis/selector_eval_20260914_142324"
CALIBRATION="/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json"
MAZE_ARP="/home/huiyuan/teleoperation/maze_real/outputs/maze_planarp_train49_7p5hz/checkpoints/epoch_0209.pt"
THREADING_ARP="/home/huiyuan/teleoperation/threading_real/outputs/threading_combined_80_mvt_cam1_planarp_v2/20260912_171342/checkpoints/epoch=0004-val_loss=10.049.ckpt"
mkdir -p "$EVAL_ROOT/logs"
```

下面四条命令包含 `--execute --confirm-real-robot`，运行后会在每轮按 Enter 开始时执行真实动作。
四个条件依次运行，共用机器人和相机，不能同时启动。

## Maze：每个条件 20 次

每轮夹取完成后，记录当前 TCP 的 Z 为该轮固定高度；所有动作只累加 XY，所有目标点的
Z 始终等于该固定值，不随实测高度更新。下一轮复位、夹取后重新记录高度。
`--z-tolerance 0.01` 是实测高度误差的提示阈值，超过 1 cm 会提示但不因此退出。
日志中的 `fixed_z` / `target_z` 为目标高度，`measured_z` 为实测高度，
`z_error_m = measured_z - fixed_z`。固定的是控制目标，实际高度仍取决于控制器的跟踪效果。

Maze 默认采用**手动 guide 切换确认**，不需要设置 `MAZE_GUIDE_ENTER`、
`MAZE_GUIDE_EXIT` 或传入两个 `--guide-*-command` 参数。每轮停止运动后，程序提示在
机器人界面启用 guide，完成后按 Enter 确认；下一轮启动前再提示退出 guide 并确认。
手动确认没有 30 秒超时，可以按 Ctrl+C 退出。程序本身不会通过 RPC 自动切换 guide。

如果已有实际控制 guide 的外部命令，仍可通过 `--guide-enter-command` 和
`--guide-exit-command` 指定；外部命令保留 30 秒超时。

第一轮开始前自行确认机器人已退出 guide。每轮结束时 runner 停止运动，再提示进入 guide。
手动复位后，在主终端按 Enter 准备下一轮，按提示退出 guide 并确认；随后自动夹取并开始推理。
`--initial-grasp-width 0.02` 沿用当前夹取设置，两组保持相同。最后一轮结束后也会提示进入 guide。

**无 selector：固定执行 10 步。**

```bash
python maze_real/scripts/deployment/cartesian.py "$MAZE_ARP" \
  --weights model --device cuda:0 \
  --calibration "$CALIBRATION" \
  --policy-hz 7.5 --execute-steps 10 \
  --episodes 20 --max-cycles 0 \
  --initial-grasp-width 0.02 \
  --trace-output "$EVAL_ROOT/logs/maze_no_selector.jsonl" \
  --results-csv "$EVAL_ROOT/logs/maze_no_selector.csv" \
  --execute --confirm-real-robot
```

**有 selector：动态执行 4–10 步。**

```bash
python maze_real/scripts/deployment/cartesian.py "$MAZE_ARP" \
  --weights model --device cuda:0 \
  --calibration "$CALIBRATION" \
  --policy-hz 7.5 --execute-steps 10 \
  --selector "$EVAL_ROOT/models/maze_h4" \
  --episodes 20 --max-cycles 0 \
  --initial-grasp-width 0.02 \
  --trace-output "$EVAL_ROOT/logs/maze_selector_h4.jsonl" \
  --results-csv "$EVAL_ROOT/logs/maze_selector_h4.csv" \
  --execute --confirm-real-robot
```

有 selector 时，实际执行步数由 selector 决定，`--execute-steps 10` 不会将动态输出固定为 10。

## Threading：每个条件 20 次

每轮手动摆放并退出机器人手动引导后，按 Enter 开始；运行中按 Enter 结束当前轮。
runner 停止后再手动进入 guide 复位。Threading runner 不会自动切换 guide。

**无 selector：固定执行 10 步。**

```bash
python threading_real/scripts/deployment/cartesian.py "$THREADING_ARP" \
  --weights model --device cuda:0 \
  --pointcloud-calibration "$CALIBRATION" \
  --policy-hz 7.5 --execute-steps 10 \
  --prediction-mode full_then_truncate --synchronous \
  --episodes 20 --max-cycles 0 \
  --grasp-before-inference --initial-grasp-width 0.02 \
  --trace-output "$EVAL_ROOT/logs/threading_no_selector.jsonl" \
  --results-csv "$EVAL_ROOT/logs/threading_no_selector.csv" \
  --execute --confirm-real-robot
```

**有 selector：动态执行 4–10 步。**

```bash
python threading_real/scripts/deployment/cartesian.py "$THREADING_ARP" \
  --weights model --device cuda:0 \
  --pointcloud-calibration "$CALIBRATION" \
  --policy-hz 7.5 --execute-steps 10 \
  --chunk-selector "$EVAL_ROOT/models/threading_h4" \
  --prediction-mode full_then_truncate --synchronous \
  --episodes 20 --max-cycles 0 \
  --grasp-before-inference --initial-grasp-width 0.02 \
  --trace-output "$EVAL_ROOT/logs/threading_selector_h4.jsonl" \
  --results-csv "$EVAL_ROOT/logs/threading_selector_h4.csv" \
  --execute --confirm-real-robot
```

## 评估与记录

每轮从开始推理计时，统一给 60 秒完成任务；达到成功条件、明显失败或超时后按 Enter 结束。
60 秒需人工计时，当前命令不自动执行时限判断；`--max-cycles 0` 表示不限制推理轮数。
不使用相同的最大推理轮数作时限，因为两种执行长度对应的实际时长不同。

成功条件在第一轮前固定，两组一致：Maze 完成迷宫路径并到达预先指定的目标位置；Threading
完成预先约定的穿线终态。开始后的人工辅助、卡住、物体脱落、超时或动作检查中止记为失败，
并记录原因。不能把进入 spatial rule 精细区当作任务成功，也不能把模型的验证准确率当成功率。

20 个起始条件提前编号，每种方法均按相同编号复现。记录运行顺序；若按下面命令先做完一个
条件再做另一个，汇总时注明这个顺序，避免忽略人工熟练度或设备状态随时间变化的影响。

两大统计量为 **成功率** 和 **每轮耗时（秒）**。计时从夹取、初始化完成后的推理循环开始，
到脚本检测到结束 Enter 为止，包含观测、推理和执行等待，不包含停止机器人的等待、填写结果或手动复位。
Enter 由控制循环轮询检测；若在同步推理期间按下，会在推理返回后检测到，因此可能有检测延迟。

机器人停止后，终端提示输入 `1`（成功）或 `0`（失败），再按 Enter 确认。
Maze 填写结果后才进入 guide 确认。每次保存后显示已评估次数、成功率和平均耗时；
成功率按所有已填写结果的完整记录计算；平均耗时只统计其中成功（`success=1`）的尝试。
没有成功样本时显示“暂无成功样本”，不记为 0 秒。CSV 保留每次尝试的原始耗时，失败耗时不参与平均值。

四条命令的 `--results-csv` 分别指定四个 CSV 文件名。同名文件会读取历史记录并继续写入，
CSV 编号从已有最大编号加 1 开始，统计包含历史完整记录。补跑时沿用同一名字，
`--episodes` 指定本次新增尝试数（例如已有 8 次，再运行 `--episodes 12`）。
文件中的任务、selector 条件和真机/诊断模式必须与当前命令一致。
不传该参数时保留原运行方式，不进行交互结果记录。

CSV 列为 `episode, task, condition, executed, started_at, duration_s, success, status`。
每轮开始立即保存 `running`；检测到结束 Enter 后立即保存耗时及 `pending_result`；
填写 1/0 后保存为 `completed`。每次写入均使用临时文件、flush、fsync 和原子替换，
已保存记录不会因后续程序报错而丢失。异常退出保留 `interrupted`；达到轮数限制为 `max_cycles`。
强制终止时可能保留 `running`，未填写结果时保留 `pending_result`，这些记录不计入成功率或平均耗时。
新文件的 episode 从 1 编号；未完成记录也保留原编号，后续尝试使用新编号。
终端会显示 CSV 记录编号；runner 的 episode 提示表示本次启动内的轮次。
同一个 CSV 应由一个部署进程使用。

已有 Maze 无 selector 的 8 次手工记录没有耗时，**只保留在下面表中，不导入新 CSV**。
新的两项指标对照实验从完整计时的记录开始；下表作为历史记录，不与新 CSV 混算。

### Maze 无 selector（固定 10 步）

| 起始条件编号 | 成功（1 / 0） | 备注／失败原因 |
| --- | --- | --- |
| 01 | 1 |  |
| 02 | 0 |  |
| 03 | 1 |  |
| 04 | 0 |  |
| 05 | 1 |  |
| 06 | 0 |  |
| 07 | 1 |  |
| 08 | 1 |  |
| 09 |  |  |
| 10 |  |  |
| 11 |  |  |
| 12 |  |  |
| 13 |  |  |
| 14 |  |  |
| 15 |  |  |
| 16 |  |  |
| 17 |  |  |
| 18 |  |  |
| 19 |  |  |
| 20 |  |  |

### Maze 有 selector（动态 4–10 步）

| 起始条件编号 | 成功（1 / 0） | 备注／失败原因 |
| --- | --- | --- |
| 01 |  |  |
| 02 |  |  |
| 03 |  |  |
| 04 |  |  |
| 05 |  |  |
| 06 |  |  |
| 07 |  |  |
| 08 |  |  |
| 09 |  |  |
| 10 |  |  |
| 11 |  |  |
| 12 |  |  |
| 13 |  |  |
| 14 |  |  |
| 15 |  |  |
| 16 |  |  |
| 17 |  |  |
| 18 |  |  |
| 19 |  |  |
| 20 |  |  |

### Threading 无 selector（固定 10 步）

| 起始条件编号 | 成功（1 / 0） | 备注／失败原因 |
| --- | --- | --- |
| 01 |  |  |
| 02 |  |  |
| 03 |  |  |
| 04 |  |  |
| 05 |  |  |
| 06 |  |  |
| 07 |  |  |
| 08 |  |  |
| 09 |  |  |
| 10 |  |  |
| 11 |  |  |
| 12 |  |  |
| 13 |  |  |
| 14 |  |  |
| 15 |  |  |
| 16 |  |  |
| 17 |  |  |
| 18 |  |  |
| 19 |  |  |
| 20 |  |  |

### Threading 有 selector（动态 4–10 步）

| 起始条件编号 | 成功（1 / 0） | 备注／失败原因 |
| --- | --- | --- |
| 01 |  |  |
| 02 |  |  |
| 03 |  |  |
| 04 |  |  |
| 05 |  |  |
| 06 |  |  |
| 07 |  |  |
| 08 |  |  |
| 09 |  |  |
| 10 |  |  |
| 11 |  |  |
| 12 |  |  |
| 13 |  |  |
| 14 |  |  |
| 15 |  |  |
| 16 |  |  |
| 17 |  |  |
| 18 |  |  |
| 19 |  |  |
| 20 |  |  |

## 成功率汇总

```text
运行中成功率 = 已填写结果的成功次数 / 已填写结果的次数 × 100%
完成 20 次后的成功率 = 成功次数 / 20 × 100%
成功尝试平均耗时 = 已完成且 success=1 的各轮 duration_s 之和 / 成功次数
selector 提升（百分点） = selector 成功率 - no selector 成功率
```

每多成功 1 次，成功率增加 5 个百分点。下面待真机评估后填写，不预填模型验证结果。

| 任务 | 无 selector 成功数 / 20 | 无 selector 成功率 | 有 selector 成功数 / 20 | 有 selector 成功率 | 提升（百分点） |
| --- | --- | --- | --- | --- | --- |
| Maze | 待评估 | 待评估 | 待评估 | 待评估 | 待评估 |
| Threading | 待评估 | 待评估 | 待评估 | 待评估 | 待评估 |
