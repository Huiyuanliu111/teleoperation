# Selector 对照实验部署

Maze 和 Threading 分别比较 **no selector（固定执行 10 步）** 与 **selector（动态执行 3–10 步）**。
每个条件评估 20 次，即每个任务 40 次，两个任务共 80 次。每个任务使用同一套 20 个起始条件，
第 i 次在两种方法下尽量复现相同的物体、起始位姿和目标位置。

| 任务 | 条件 | 执行步数 | 归一化节点 | 评估次数 |
| --- | --- | --- | --- | --- |
| Maze | no selector | 固定 10 | 不使用 | 20 |
| Maze | selector | 3–10 | 30% | 20 |
| Threading | no selector | 固定 10 | 不使用 | 20 |
| Threading | selector | 3–10 | 50% | 20 |

两组均生成完整的 10 步动作预测，再执行指定长度的前缀。同一任务使用相同的 ARP checkpoint、
`model` 权重、7.5 Hz 控制频率、相机标定及动作限制，仅改变执行长度是否由 selector 决定。
不启用 endpoint schedule 或额外的低置信度回退。

## 本次模型版本

本次已将两个当前最佳 selector 复制成固定评估版本，训练继续更新不会改变本次实验使用的模型。
Threading 快照为 epoch 7（训练仍在继续）；Maze 快照为 epoch 7（已早停）。
如果要评估后续模型，应建立新的一轮实验，不能在同一组 20 次中途换权重。

评估目录：`data/analysis/selector_eval_20260914_142324`。
[模型来源、epoch 与 SHA-256 清单](../data/analysis/selector_eval_20260914_142324/manifest.json)。

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

当前 Maze runner 要求提供 guide 进入／退出命令。以下采用**人工在机器人界面切换 guide，
再按 Enter 确认**的方式；这些命令只等待确认，不会通过 RPC 自动切换 guide。
每个确认提示有 runner 设置的 30 秒超时，完成实际切换后再确认。

```bash
MAZE_GUIDE_ENTER='python -c "input(\"请在机器人界面启用手动引导，完成后按 Enter：\")"'
MAZE_GUIDE_EXIT='python -c "input(\"请退出手动引导，完成后按 Enter：\")"'
```

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
  --guide-enter-command "$MAZE_GUIDE_ENTER" \
  --guide-exit-command "$MAZE_GUIDE_EXIT" \
  --trace-output "$EVAL_ROOT/logs/maze_no_selector.jsonl" \
  --execute --confirm-real-robot
```

**有 selector：动态执行 3–10 步。**

```bash
python maze_real/scripts/deployment/cartesian.py "$MAZE_ARP" \
  --weights model --device cuda:0 \
  --calibration "$CALIBRATION" \
  --policy-hz 7.5 --execute-steps 10 \
  --selector "$EVAL_ROOT/models/maze" \
  --episodes 20 --max-cycles 0 \
  --initial-grasp-width 0.02 \
  --guide-enter-command "$MAZE_GUIDE_ENTER" \
  --guide-exit-command "$MAZE_GUIDE_EXIT" \
  --trace-output "$EVAL_ROOT/logs/maze_selector.jsonl" \
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
  --execute --confirm-real-robot
```

**有 selector：动态执行 3–10 步。**

```bash
python threading_real/scripts/deployment/cartesian.py "$THREADING_ARP" \
  --weights model --device cuda:0 \
  --pointcloud-calibration "$CALIBRATION" \
  --policy-hz 7.5 --execute-steps 10 \
  --chunk-selector "$EVAL_ROOT/models/threading" \
  --prediction-mode full_then_truncate --synchronous \
  --episodes 20 --max-cycles 0 \
  --grasp-before-inference --initial-grasp-width 0.02 \
  --trace-output "$EVAL_ROOT/logs/threading_selector.jsonl" \
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

`--episodes 20` 只控制运行次数，JSONL 只记录执行信息，**不会自动判断任务成功**。
在下表填 `1`（成功）或 `0`（失败）；未测试保持空白。中途退出时保留已记录结果，补跑剩余
次数并另存日志，避免重复计数。各条件最终均应有 20 次完整记录。

[CSV 记录模板（80 行，可用表格软件填写）](../data/analysis/selector_eval_20260914_142324/results.csv)。

| 起始条件编号 | Maze 无 selector | Maze 有 selector | Threading 无 selector | Threading 有 selector | 备注／失败原因 |
| --- | --- | --- | --- | --- | --- |
| 01 | | | | | |
| 02 | | | | | |
| 03 | | | | | |
| 04 | | | | | |
| 05 | | | | | |
| 06 | | | | | |
| 07 | | | | | |
| 08 | | | | | |
| 09 | | | | | |
| 10 | | | | | |
| 11 | | | | | |
| 12 | | | | | |
| 13 | | | | | |
| 14 | | | | | |
| 15 | | | | | |
| 16 | | | | | |
| 17 | | | | | |
| 18 | | | | | |
| 19 | | | | | |
| 20 | | | | | |

## 成功率汇总

```text
每个条件的成功率 = 成功次数 / 20 × 100%
selector 提升（百分点） = selector 成功率 - no selector 成功率
```

每多成功 1 次，成功率增加 5 个百分点。下面待真机评估后填写，不预填模型验证结果。

| 任务 | 无 selector 成功数 / 20 | 无 selector 成功率 | 有 selector 成功数 / 20 | 有 selector 成功率 | 提升（百分点） |
| --- | --- | --- | --- | --- | --- |
| Maze | 待评估 | 待评估 | 待评估 | 待评估 | 待评估 |
| Threading | 待评估 | 待评估 | 待评估 | 待评估 | 待评估 |
