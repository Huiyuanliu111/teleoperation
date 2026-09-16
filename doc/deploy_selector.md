# Selector 对照实验部署

本页命令使用 2026-09-16 重训的 selector，与提取训练特征时的动作模型严格配对。
Maze 比较 **固定 50 步** 与 **动态 20–50 步**；Threading 比较 **固定 20 步** 与 **动态 8–20 步**。
每个条件评估 20 次，即每个任务 40 次，两个任务共 80 次。每个任务使用同一套 20 个起始条件，
第 i 次在两种方法下尽量复现相同的物体、起始位姿和目标位置。

| 任务 | 条件 | 执行步数 | 归一化节点 | 评估次数 |
| --- | --- | --- | --- | --- |
| Maze | no selector | 固定 50 | 不使用 | 20 |
| Maze | selector | 20–50 | 路程 30%，先细后粗 | 20 |
| Threading | no selector | 固定 20 | 不使用 | 20 |
| Threading | selector | 8–20 | 路程 50%，先粗后细 | 20 |

Maze 两组均生成完整 50 步预测，Threading 两组均生成完整 20 步预测，再执行指定长度的前缀。同一任务使用相同的 ARP checkpoint、
`model` 权重、7.5 Hz 控制频率、相机标定及动作限制，仅改变执行长度是否由 selector 决定。
不启用 endpoint schedule 或额外的低置信度回退。

## 本次模型版本

| 任务 | 动作模型 | selector 最佳权重 | 执行范围 |
| --- | --- | --- | --- |
| Threading | H20，epoch 8，val_loss=10.981 | 路程标签重训，epoch 11 | 8–20 |
| Maze | H50，epoch 9 | 路程标签重训，epoch 5 | 20–50 |

Threading 已按用户要求停止训练，Maze 已完成训练；以下模型目录都已生成最佳权重。
两者使用各自部署动作模型的 `model` 编码器重新提取特征，不能替换为旧 H10 selector，
也不能仅修改候选长度后迁移到其他动作模型 checkpoint。

精细区概率为 p：Threading 执行 `floor(8*p + 20*(1-p) + 0.5)`，
Maze 执行 `floor(20*p + 50*(1-p) + 0.5)`。标签按每条轨迹的 TCP 累计路程生成：
Threading 45%–55% 从粗区过渡到细区；Maze 25%–35% 从细区过渡到粗区。
路程进度仅用于离线标签，在线仍由视觉 selector 预测，不能把标签切换当成已验证的真机行为。

新评估目录：`data/analysis/selector_eval_matched_20260916`。
[模型路径与 SHA-256 清单](../data/analysis/selector_eval_matched_20260916/manifest.json)。
新命令使用独立日志，避免与旧 H10/h4 或编码器不匹配的 h8/H20 实验混算。
训练记录：[Threading](Selector_H20_progress_training.md)；Maze 运行目录为
`training_runs/maze_selector_H50_h20_progress_20260916_140818`。

## 运行环境

下面四组部署命令均使用完整路径，可单独复制，不依赖 `MAZE_ARP` 等 shell 变量。
在交互式终端中运行以下准备命令。部署使用 `pushbox` 环境：本机该环境具备 Pinocchio 和
RealSense 依赖，训练使用的 `arp` 环境缺少这两个部署依赖。

```bash
cd /home/huiyuan/teleoperation
source /home/huiyuan/miniconda3/etc/profile.d/conda.sh
conda activate pushbox

mkdir -p "/home/huiyuan/teleoperation/data/analysis/selector_eval_matched_20260916/logs"
```

下面四条命令包含 `--execute --confirm-real-robot`，运行后会在每轮按 Enter 开始时执行真实动作。
四个条件依次运行，共用机器人和相机，不能同时启动。

## Maze：每个条件 20 次

H50 在 7.5 Hz 下最长执行时间约 6.67 s；以下两组命令使用 `--sync-timeout 10.0`，
避免默认 3 s 超时在长 chunk 执行完成前中止等待。

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

**无 selector：固定执行 50 步。**

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/maze_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/training_runs/planarp_chunks_20260914_165545/maze_planarp_chunk50/checkpoints/epoch_0009.pt" \
  --weights model --device cuda:0 \
  --calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 50 --sync-timeout 10.0 \
  --episodes 20 --max-cycles 0 \
  --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_matched_20260916/logs/maze50_no_selector.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_matched_20260916/logs/maze50_no_selector.csv" \
  --execute --confirm-real-robot
```

**有 selector：动态执行 20–50 步。**

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/maze_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/training_runs/planarp_chunks_20260914_165545/maze_planarp_chunk50/checkpoints/epoch_0009.pt" \
  --weights model --device cuda:0 \
  --calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 50 --sync-timeout 10.0 \
  --selector "/home/huiyuan/teleoperation/training_runs/maze_selector_H50_h20_progress_20260916_140818/model" \
  --episodes 20 --max-cycles 0 \
  --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_matched_20260916/logs/maze50_selector_h20_progress.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_matched_20260916/logs/maze50_selector_h20_progress.csv" \
  --execute --confirm-real-robot
```

有 selector 时，实际执行步数由 selector 决定，`--execute-steps 50` 不会将动态输出固定为 50。

## Threading：每个条件 20 次

每轮手动摆放并退出机器人手动引导后，按 Enter 开始；运行中按 Enter 结束当前轮。
runner 停止后再手动进入 guide 复位。Threading runner 不会自动切换 guide。

H20 在 7.5 Hz 下最长执行时间为 `20/7.5 ≈ 2.67 s`，超过默认同步超时 2 s。
以下两组命令均显式使用 `--sync-timeout 5.0`，为执行和收敛等待留出余量。

**无 selector：固定执行 20 步。**

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/threading_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/training_runs/planarp_chunks_20260914_165545/threading_planarp_chunk20/checkpoints/epoch=0008-val_loss=10.981.ckpt" \
  --weights model --device cuda:0 \
  --pointcloud-calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 20 \
  --prediction-mode full_then_truncate --synchronous --sync-timeout 5.0 \
  --episodes 20 --max-cycles 0 \
  --grasp-before-inference --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_matched_20260916/logs/threading20_no_selector.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_matched_20260916/logs/threading20_no_selector.csv" \
  --execute --confirm-real-robot
```

**有 selector：动态执行 8–20 步。**

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/threading_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/training_runs/planarp_chunks_20260914_165545/threading_planarp_chunk20/checkpoints/epoch=0008-val_loss=10.981.ckpt" \
  --weights model --device cuda:0 \
  --pointcloud-calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 20 \
  --chunk-selector "/home/huiyuan/teleoperation/training_runs/threading_selector_H20_h8_progress_20260916_134300/model" \
  --prediction-mode full_then_truncate --synchronous --sync-timeout 5.0 \
  --episodes 20 --max-cycles 0 \
  --grasp-before-inference --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_matched_20260916/logs/threading20_selector_h8_progress.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_matched_20260916/logs/threading20_selector_h8_progress.csv" \
  --execute --confirm-real-robot
```

## 评估与记录

每轮从开始推理计时，统一给 60 秒完成任务；达到成功条件、明显失败或超时后按 Enter 结束。
60 秒需人工计时，当前命令不自动执行时限判断；`--max-cycles 0` 表示不限制推理轮数。
不使用相同的最大推理轮数作时限，因为两种执行长度对应的实际时长不同。

成功条件在第一轮前固定，两组一致：Maze 完成迷宫路径并到达预先指定的目标位置；Threading
完成预先约定的穿线终态。开始后的人工辅助、卡住、物体脱落、超时或动作检查中止记为失败，
并记录原因。不能把 selector 判为精细区当作任务成功，也不能把模型的验证准确率当成功率。

20 个起始条件提前编号，每种方法均按相同编号复现。记录运行顺序；若按下面命令先做完一个
条件再做另一个，汇总时注明这个顺序，避免忽略人工熟练度或设备状态随时间变化的影响。

两大统计量为 **成功率** 和 **每轮耗时（秒）**。计时从夹取、初始化完成后的推理循环开始，
到脚本检测到结束 Enter 为止，包含观测、推理和执行等待，不包含停止机器人的等待、填写结果或手动复位。
Enter 由控制循环轮询检测；若在同步推理期间按下，会在推理返回后检测到，因此可能有检测延迟。

机器人停止后，终端提示输入 `1`（成功）或 `0`（失败），再按 Enter 确认。
Maze 填写结果后才进入 guide 确认。每次保存后显示已评估次数、成功率和平均耗时；
本报告成功率按 `completed` 的有效结果加未被明确排除的 `interrupted` 记录计算，`interrupted` 计失败（下文明确排除的记录除外）；
平均耗时仅统计 `completed` 且 `success=1` 的尝试。部署终端现有统计仍使用旧口径，可能与本报告不同。
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

此前另有 Maze 无 selector 的 8 次无耗时手工记录，结果依次为 `1,0,1,0,1,0,1,1`。
这些记录单独保留，不与下面四份计时 CSV 合并。以下编号为各 CSV 的原始 `episode`，
不是跨条件配对的起始条件编号。按用户更新的口径，`interrupted` 计作失败，即使原 `success` 为空。
原始 CSV 不修改，表中的中断结果 0 是统计时赋值。
按用户指定，Threading 无 selector 的 episode 18 从下表和统计中排除，其他记录保留原编号。

### Maze H10 无 selector（固定 10 步）

来源：[CSV](../data/analysis/selector_eval_20260914_142324/logs/maze_no_selector.csv)。

| CSV episode | 统计结果（1 / 0） | 耗时（秒） | 状态 / 计入口径 |
| --- | --- | --- | --- |
| 1 | 1 | 11.694 | completed |
| 2 | 0 | 9.955 | completed |
| 3 | 1 | 9.818 | completed |
| 4 | 1 | 9.670 | completed |
| 5 | 1 | 13.160 | completed |
| 6 | 1 | 10.676 | completed |
| 7 | 0 | 4.967 | completed |
| 8 | 0 | 5.359 | interrupted，计失败 |
| 9 | 1 | 11.437 | completed |
| 10 | 0 | 3.984 | completed |
| 11 | 1 | 9.780 | completed |
| 12 | 1 | 13.568 | completed |
| 13 | 0 | 5.311 | interrupted，计失败 |
| 14 | 1 | 14.040 | completed |
| 15 | 1 | 9.786 | completed |
| 16 | 1 | 12.584 | completed |
| 17 | 1 | 12.877 | completed |
| 18 | 1 | 12.685 | completed |
| 19 | 0 | 4.414 | completed |
| 20 | 0 | 4.561 | completed |

### Maze H10 有 selector（动态 4–10 步）

来源：[CSV](../data/analysis/selector_eval_20260914_142324/logs/maze10_selector_h4.csv)。

| CSV episode | 统计结果（1 / 0） | 耗时（秒） | 状态 / 计入口径 |
| --- | --- | --- | --- |
| 1 | 1 | 16.073 | completed |
| 2 | 0 | 10.026 | completed |
| 3 | 1 | 17.286 | completed |
| 4 | 1 | 17.359 | completed |
| 5 | 1 | 11.870 | completed |
| 6 | 1 | 16.164 | completed |
| 7 | 1 | 14.395 | completed |
| 8 | 0 | 14.810 | completed |
| 9 | 1 | 20.629 | completed |
| 10 | 0 | 9.229 | completed |
| 11 | 0 | 17.452 | completed |
| 12 | 1 | 25.228 | completed |
| 13 | 1 | 14.749 | completed |
| 14 | 0 | 11.129 | completed |
| 15 | 1 | 13.296 | completed |
| 16 | 1 | 12.780 | completed |
| 17 | 1 | 16.737 | completed |
| 18 | 1 | 57.502 | completed |
| 19 | 1 | 13.587 | completed |
| 20 | 1 | 14.549 | completed |

### Threading H20 无 selector（固定 20 步）

来源：[CSV](../data/analysis/selector_eval_chunks_20260916/logs/threading20_no_selector.csv)。

| CSV episode | 统计结果（1 / 0） | 耗时（秒） | 状态 / 计入口径 |
| --- | --- | --- | --- |
| 1 | 0 | 15.284 | completed |
| 2 | 1 | 14.379 | completed |
| 3 | 1 | 12.853 | completed |
| 4 | 0 | 15.703 | completed |
| 5 | 1 | 12.787 | completed |
| 6 | 1 | 14.820 | completed |
| 7 | 0 | 16.125 | completed |
| 8 | 0 | 13.781 | completed |
| 9 | 1 | 14.469 | completed |
| 10 | 0 | 16.531 | completed |
| 11 | 1 | 14.473 | completed |
| 12 | 0 | 14.023 | completed |
| 13 | 0 | 11.530 | completed |
| 14 | 0 | 10.483 | completed |
| 15 | 0 | 17.747 | completed |
| 16 | 1 | 17.733 | completed |
| 17 | 0 | 19.983 | completed |
| 19 | 1 | 15.390 | completed |
| 20 | 0 | 14.770 | completed |
| 21 | 1 | 17.712 | completed |

### Threading H20 新 selector（动态 8–20 步）

来源：[CSV](../data/analysis/selector_eval_matched_20260916/logs/threading20_selector_h8_progress.csv)。

| CSV episode | 统计结果（1 / 0） | 耗时（秒） | 状态 / 计入口径 |
| --- | --- | --- | --- |
| 1 | 1 | 20.015 | completed |
| 2 | 0 | 20.109 | completed |
| 3 | 1 | 20.252 | completed |
| 4 | 1 | 16.007 | completed |
| 5 | 0 | 13.251 | completed |
| 6 | 1 | 12.300 | completed |
| 7 | 0 | 13.834 | completed |
| 8 | 1 | 19.768 | completed |
| 9 | 0 | 15.692 | completed |
| 10 | 1 | 18.804 | completed |
| 11 | 0 | 19.159 | completed |
| 12 | 0 | 15.313 | completed |
| 13 | 0 | 17.895 | completed |
| 14 | 1 | 19.329 | completed |
| 15 | 1 | 17.799 | completed |
| 16 | 1 | 16.119 | completed |
| 17 | 1 | 14.924 | completed |
| 18 | 1 | 17.150 | completed |
| 19 | 1 | 17.131 | completed |
| 20 | 1 | 19.239 | completed |

## 成功率与耗时汇总（2026-09-16 更新）

- 成功率分母：`completed` 且 `success` 为 0/1 的记录，加未被明确排除的 `interrupted` 记录。
- 成功率分子：`completed` 且 `success=1` 的记录数。`interrupted` 计失败（下文明确排除的记录除外）。
- 成功平均耗时：仅统计上述成功记录；失败和中断不进入耗时均值。
- `running`、`pending_result`、未填写结果的其他状态暂不计入。

| 任务 / 条件 | 计入次数 | 中断失败数 | 成功数 | 失败数（含中断） | 成功率 | 成功平均耗时 |
| --- | --- | --- | --- | --- | --- | --- |
| Maze H10 无 selector（固定 10 步） | 20 | 2 | 13 | 7 | 65.00% | 11.68 s |
| Maze H10 有 selector（动态 4–10 步） | 20 | 0 | 15 | 5 | 75.00% | 18.81 s |
| Threading H20 无 selector（固定 20 步） | 20 | 0 | 9 | 11 | 45.00% | 14.96 s |
| Threading H20 新 selector（动态 8–20 步） | 20 | 0 | 13 | 7 | 65.00% | 17.60 s |

Maze 无 selector 的 episode 8、13 计失败，因此为 **13/20（65%）**，不再是 13/18。
Threading 无 selector 按用户指定排除 episode 18，仅统计其余 20 条记录，结果为 **9/20（45%）**。
这一例外不影响 Maze 的中断计失败口径，成功平均耗时不变。

| 对比 | 无 selector → 有 selector 成功率 | 提升 | 成功平均耗时变化 |
| --- | --- | --- | --- |
| Maze H10 | 65.00% → 75.00% | +10.00 个百分点 | 11.68 s → 18.81 s（+7.14 s） |
| Threading H20（新 selector） | 45.00% → 65.00% | +20.00 个百分点 | 14.96 s → 17.60 s（+2.65 s） |

Maze selector 的 episode 18 成功耗时为 57.502379 s，按原记录保留在均值中，未剔除。
以上 Maze 结果属于 H10，不能填入 H50 的结果。耗时仅比较各组成功样本，不是全部尝试的平均差。
无 selector 基线来自早前运行；CSV 没有 checkpoint 或起始条件配对标识，因此只报告观察到的组间差异，
不由 CSV 推断模型版本和起始条件完全一致。下方旧 Threading selector 与本次重训版本单独列示。

## Selector chunk 分布（统一绘图样式）

两图统一使用此前 Threading 的折线图风格：左侧用带圆点的折线展示每条 trace 的实际执行 chunk，
右侧用蓝色柱状图展示决策次数，柱顶标注次数和百分比。两图使用相同尺寸、字体和布局。
折线颜色区分 trace，图例位于下方；线条只覆盖实际记录，不补齐结束后的决策。
横轴为决策序号，不是时间或路程，各任务保留实际序列长度。
trace 遇到 `cycle=1` 重新编号，以免 runner 重启后重复的 episode 编号混在一起。

### Maze H10 / selector h4

20 段 trace、344 次决策：4 步 180 次（52.3%），10 步 130 次（37.8%），
5–9 步共 34 次（9.9%）。19 段到达 10 步后均未缩短，总体呈现先短后长。

![Maze H10 selector h4 chunk distribution](../data/analysis/selector_eval_20260914_142324/logs/maze10_selector_h4_chunk_distribution.png)

### Threading H20 / selector h8

20 段 trace、168 次决策：8 步 106 次（63.1%），20 步 48 次（28.6%），
中间长度共 14 次（8.3%）。全部 trace 均从 20 步开始，于第 3–5 次决策切换至 8 步后保持。

![Threading H20 selector h8 chunk distribution](../data/analysis/selector_eval_matched_20260916/logs/threading20_selector_h8_progress_chunk_distribution.png)

Maze 原先含稀疏 TCP XY 路程估算的三联图[单独保留](../data/analysis/selector_eval_20260914_142324/logs/maze10_selector_h4_with_sparse_path.png)。
该路程估算缺少 chunk 内采样和最终实测终点，不与 Threading 决策序号图混作路程比较。

重新生成上述两张统一样式图片（从仓库根目录执行）：

```bash
/home/huiyuan/miniconda3/envs/arp/bin/python threading_real/scripts/chunk_selector/plot_matched_chunk_distributions.py
```

## 历史结果：2026-09-16 Threading 20 步（旧 selector）

以下使用旧 H10 特征训练并重映射的 selector，不是本页最新配对重训版本；保留作历史记录。

历史结果来自 `data/analysis/selector_eval_chunks_20260916/logs`。原文记录两组均有 20 次 completed
和 1 次 interrupted；无 selector 按用户要求排除 episode 18，计 20 次；旧 selector 仍计 21 次，
其中断计失败。旧 selector CSV 当前不在该目录，
其成功数和平均耗时沿用此前已记录结果，分母依据上述历史记录更新，未重新核验缺失的 CSV。

| 条件 | 成功数 / 计入次数 | 成功率 | 成功尝试平均耗时 |
| --- | --- | --- | --- |
| no selector（固定 20 步） | 9 / 20 | 45.00% | 14.96 s |
| 旧 selector h8（动态 8–20 步） | 15 / 21 | 71.43% | 19.48 s |

按上述口径，旧 selector 的成功率高 26.43 个百分点。耗时平均值仅统计成功尝试，符合上文定义；它不表示全部尝试的平均时长。

### selector chunk 分布

`threading20_selector_h8.jsonl` 现有 23 个 trace 序列、282 次 selector 决策。trace 文件以追加方式
跨多次 runner 启动写入，runner 每次启动会重置 `episode` 编号，因此以每个 `cycle=1` 为新序列边界。
实际执行 chunk 以 8 步为主（261 次，92.55%）；其余为 13 步 2 次（0.71%）、14 步 2 次（0.71%）、
15 步 3 次（1.06%）、16 步 11 次（3.90%）、17 步 3 次（1.06%）；没有 20 步。平均实际 chunk 为 8.54 步。
归一化前半段的 147 次决策中，8 步有 131 次（89.12%）；后半段的 135 次决策中，8 步有 130 次（96.30%）。

图中将每个 trace 序列从首个到末个 selector 决策的时间线归一化到 0–100%，再分为 10 个等宽时间段；
每个点的频率是该时间段中对应 chunk 的决策次数除以该段全部决策次数。它描述这 23 个已有 trace 序列的
决策分布，并不将缺失 trace 的其余完成评估轮推断进图中。原始分箱统计见
[CSV](../data/analysis/selector_eval_chunks_20260916/logs/threading20_selector_h8_chunk_frequency_by_normalized_time.csv)。

![Threading selector chunk frequency over normalized time](../data/analysis/selector_eval_chunks_20260916/logs/threading20_selector_h8_chunk_frequency_by_normalized_time.png)
