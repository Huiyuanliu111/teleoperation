# 固定短执行长度对比实验

本实验是 [Selector 对照实验部署](deploy_selector.md) 的扩充，新增两个任务的 **20%、40%、60%、80% 固定执行比例**。比例指每次执行步数占策略完整预测 horizon 的比例，不是训练数据比例或任务进度。

沿用 Maze H10（epoch 209）和 Threading H20（epoch 8）策略，每次仍生成完整 H10 / H20 预测，只执行前缀后重新观测、推理。所有新增条件均不启用 selector、AAC、AutoHorizon 或 execution schedule，不重新训练短 horizon 模型。

`timeout`（超时）和 `interrupted`（中断）均计为失败，纳入成功率分母。

## 比较总表（2026-09-22）

原实验数据作为历史参照，数值及数据来源沿用 [原实验总表](deploy_selector.md)。新增 8 组计划各评估 20 次，每个任务新增 80 次，共计划新增 **160 次**。当前 Maze 已计入 44 次：20%、40%、60%、80% 分别为 3、11、10、20 次。Threading 已计入 48 次：四组分别为 8、10、10、20 次。两个任务的前三组均为阶段性结果，80% 组已完成 20 次。`—` 表示暂无结果或无成功样本，不代表 0% 成功率。

| 任务 / 条件 | 计入 / 计划次数 | 中断失败数 | 成功数 | 失败数（含中断、超时） | 成功率 | 成功平均耗时 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| [Maze H10 固定 20%（2 步）](../data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p20_h2.csv) | 3/20 | 0 | 0 | 3 | 0.00% | — |
| [Maze H10 固定 40%（4 步）](../data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p40_h4.csv) | 11/20 | 0 | 5 | 6 | 45.45% | 32.37 s |
| [Maze H10 固定 60%（6 步）](../data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p60_h6.csv) | 10/20 | 0 | 7 | 3 | 70.00% | 16.15 s |
| [Maze H10 固定 80%（8 步）](../data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p80_h8.csv) | 20/20 | 0 | 14 | 6 | 70.00% | 14.65 s |
| 历史参照：[Maze H10 无 selector（固定 10 步）](../data/analysis/selector_eval_20260914_142324/logs/maze_no_selector.csv) | 20/20 | 2 | 13 | 7 | 65.00% | 11.68 s |
| 历史参照：[Maze H10 selector（4–10 步）](../data/analysis/selector_eval_20260914_142324/logs/maze10_selector_h4.csv) | 20/20 | 0 | 15 | 5 | 75.00% | 18.81 s |
| [Threading H20 固定 20%（4 步）](../data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p20_h4.csv) | 8/20 | 1 | 0 | 8 | 0.00% | — |
| [Threading H20 固定 40%（8 步）](../data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p40_h8.csv) | 10/20 | 2 | 5 | 5 | 50.00% | 18.78 s |
| [Threading H20 固定 60%（12 步）](../data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p60_h12.csv) | 10/20 | 0 | 6 | 4 | 60.00% | 20.31 s |
| [Threading H20 固定 80%（16 步）](../data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p80_h16.csv) | 20/20 | 0 | 10 | 10 | 50.00% | 17.71 s |
| 历史参照：[Threading H20 无 selector（固定 20 步）](../data/analysis/selector_eval_chunks_20260916/logs/threading20_no_selector.csv) | 20/20 | 0 | 9 | 11 | 45.00% | 14.96 s |
| 历史参照：[Threading H20 新 selector（8–20 步）](../data/analysis/selector_eval_matched_20260916/logs/threading20_selector_h8_progress.csv) | 20/20 | 0 | 13 | 7 | 65.00% | 17.60 s |

本批 Maze 20% 的 3 次尝试全部为 `timeout`；40% 的 episode 1 为 `timeout`，其余 10 次为 `completed`；60% 和 80% 全部为 `completed`，没有中断记录。4 次超时耗时均约 50 秒（50.005–50.069 s），按日志状态计失败并保留原始耗时。成功平均耗时仅统计成功的 completed 记录。已核对四组 JSONL 中已执行动作的 `executed_steps` 分别为 2、4、6、8。

本批 Threading 20% 的 episode 1 为 `timeout`（30.006 s），episode 8 为 `interrupted`；40% 的 episode 5、6 为 `interrupted`；其余记录均为 `completed`。80% 组按最新 CSV 的 20 条记录重新统计，不沿用此前 11 条记录的统计结果。四组共 48 次，成功 21 次、失败 27 次（含 1 次超时、3 次中断），合计成功率 43.75%，成功平均耗时 18.71 s。当前 60% 组成功率最高（6/10），80% 组成功平均耗时最短（17.71 s）；各组样本数不同，尚不足以确定最佳执行比例。

历史无 selector 条件即固定 100%：Maze 10 步、Threading 20 步。Threading 历史无 selector 排除 episode 18；该例外仅适用于原数据，不应用于新增组。历史 AAC 和 AutoHorizon 中未满 20 次的组仍为阶段性结果。历史数据缺少配对起始条件标识，不将新旧组间差异解释为严格配对的因果效果。

## 实验设置

| 任务 | 完整预测长度 | 固定 20% | 固定 40% | 固定 60% | 固定 80% | 各条件次数 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Maze | 10 步 | 2 步 | 4 步 | 6 步 | 8 步 | 20 |
| Threading | 20 步 | 4 步 | 8 步 | 12 步 | 16 步 | 20 |

同一任务保持 checkpoint、`model` 权重、7.5 Hz 控制频率、相机标定、夹取设置和动作限制一致，仅改变固定执行步数。Threading 显式指定 `full_then_truncate`；Maze 在完整 `predict_action` 后截取执行前缀。Maze 同步超时保持 3 秒，Threading 保持 5 秒。

各任务提前编号 20 个起始条件，四种比例按相同编号尽量复现物体、起始位姿及目标位置，另行记录起始条件编号与 CSV episode 的对应关系和实际运行顺序。CSV episode 是日志编号，不自动代表配对编号。

## 运行准备

以下命令可分别复制执行，使用 `pushbox` 部署环境。各条件共用机器人和相机，依次运行。命令包含 `--execute --confirm-real-robot`，每轮按 Enter 开始后会执行真实动作。

```bash
cd /home/huiyuan/teleoperation
source /home/huiyuan/miniconda3/etc/profile.d/conda.sh
conda activate pushbox
mkdir -p "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs"
```

新增实验的 CSV 与 JSONL 均单独保存到上述目录，文件名同时标记任务、完整 horizon、执行比例及固定步数。所有新增组的 CSV condition 都是无 selector 条件，因此需要通过文件名区分执行比例，不能混写到同一个 CSV。

## Maze H10 部署命令

沿用原实验的固定 Z、仅累加 XY 控制与手动 guide 确认流程。第一轮前确认已退出 guide；每轮自动夹取后记录固定高度，结束并填写结果后按提示进入 guide，手动复位，下一轮按提示退出 guide。初始夹取宽度保持 0.02 m。

### 固定 20%：执行 2 步

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/maze_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/maze_real/outputs/maze_planarp_train49_7p5hz/checkpoints/epoch_0209.pt" \
  --weights model --device cuda:0 \
  --calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 2 --sync-timeout 3.0 \
  --episodes 20 --max-cycles 0 \
  --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p20_h2.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p20_h2.csv" \
  --execute --confirm-real-robot
```

### 固定 40%：执行 4 步

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/maze_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/maze_real/outputs/maze_planarp_train49_7p5hz/checkpoints/epoch_0209.pt" \
  --weights model --device cuda:0 \
  --calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 4 --sync-timeout 3.0 \
  --episodes 20 --max-cycles 0 \
  --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p40_h4.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p40_h4.csv" \
  --execute --confirm-real-robot
```

### 固定 60%：执行 6 步

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/maze_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/maze_real/outputs/maze_planarp_train49_7p5hz/checkpoints/epoch_0209.pt" \
  --weights model --device cuda:0 \
  --calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 6 --sync-timeout 3.0 \
  --episodes 20 --max-cycles 0 \
  --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p60_h6.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p60_h6.csv" \
  --execute --confirm-real-robot
```

### 固定 80%：执行 8 步

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/maze_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/maze_real/outputs/maze_planarp_train49_7p5hz/checkpoints/epoch_0209.pt" \
  --weights model --device cuda:0 \
  --calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 8 --sync-timeout 3.0 \
  --episodes 20 --max-cycles 0 \
  --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p80_h8.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/maze10_fixed_p80_h8.csv" \
  --execute --confirm-real-robot
```

## Threading H20 部署命令

沿用单相机 PlanARP H20 epoch 8 checkpoint。每轮手动摆放并退出 guide 后按 Enter 开始；运行中按 Enter 结束当前轮，runner 停止后再手动进入 guide 复位。Threading runner 不会自动切换 guide。

### 固定 20%：执行 4 步

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/threading_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/training_runs/planarp_chunks_20260914_165545/threading_planarp_chunk20/checkpoints/epoch=0008-val_loss=10.981.ckpt" \
  --weights model --device cuda:0 \
  --pointcloud-calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 4 \
  --prediction-mode full_then_truncate --synchronous --sync-timeout 5.0 \
  --episodes 20 --max-cycles 0 \
  --grasp-before-inference --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p20_h4.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p20_h4.csv" \
  --execute --confirm-real-robot
```

### 固定 40%：执行 8 步

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/threading_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/training_runs/planarp_chunks_20260914_165545/threading_planarp_chunk20/checkpoints/epoch=0008-val_loss=10.981.ckpt" \
  --weights model --device cuda:0 \
  --pointcloud-calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 8 \
  --prediction-mode full_then_truncate --synchronous --sync-timeout 5.0 \
  --episodes 20 --max-cycles 0 \
  --grasp-before-inference --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p40_h8.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p40_h8.csv" \
  --execute --confirm-real-robot
```

### 固定 60%：执行 12 步

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/threading_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/training_runs/planarp_chunks_20260914_165545/threading_planarp_chunk20/checkpoints/epoch=0008-val_loss=10.981.ckpt" \
  --weights model --device cuda:0 \
  --pointcloud-calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 12 \
  --prediction-mode full_then_truncate --synchronous --sync-timeout 5.0 \
  --episodes 20 --max-cycles 0 \
  --grasp-before-inference --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p60_h12.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p60_h12.csv" \
  --execute --confirm-real-robot
```

### 固定 80%：执行 16 步

```bash
/home/huiyuan/miniconda3/envs/pushbox/bin/python /home/huiyuan/teleoperation/threading_real/scripts/deployment/cartesian.py "/home/huiyuan/teleoperation/training_runs/planarp_chunks_20260914_165545/threading_planarp_chunk20/checkpoints/epoch=0008-val_loss=10.981.ckpt" \
  --weights model --device cuda:0 \
  --pointcloud-calibration "/home/huiyuan/teleoperation/threading_real/calibration/block_grasp_spatial.json" \
  --policy-hz 7.5 --execute-steps 16 \
  --prediction-mode full_then_truncate --synchronous --sync-timeout 5.0 \
  --episodes 20 --max-cycles 0 \
  --grasp-before-inference --initial-grasp-width 0.02 \
  --trace-output "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p80_h16.jsonl" \
  --results-csv "/home/huiyuan/teleoperation/data/analysis/selector_eval_short_horizon_20260922/logs/threading20_fixed_p80_h16.csv" \
  --execute --confirm-real-robot
```

## 评估与结果填写

沿用原实验成功条件：Maze 完成路径并到达指定目标；Threading 完成预先约定的穿线终态。当前每轮自动超时默认设为 Maze 50 秒、Threading 30 秒，可用 `--episode-timeout` 覆盖，替代原计划的人工 60 秒时限。成功或明显失败后按 Enter 结束，机器人停止后输入 `1`（成功）或 `0`（失败）；自动超时记为 `timeout`、`success=0`，无需填写结果。人工辅助、卡住、物体脱落、超时或动作检查中止均记失败。`--max-cycles 0` 仅取消推理轮数限制，不关闭每轮超时。正在执行的相机读取或推理返回后才处理超时，实际停止可能晚于设定值。

总表按以下统一口径填写：

- 计入次数 = 有效 `completed`（已填写 0/1）+ `interrupted` + `timeout`；中断和超时均计失败，失败数包含二者；“中断失败数”列仅统计 `interrupted`。
- 成功率 = 成功数 / 计入次数；`running`、`pending_result` 等未完成记录暂不计入，不按缺号补失败。
- 成功平均耗时仅统计 `completed` 且 `success=1` 的 `duration_s`，无成功样本填 `—`。计时从夹取、初始化后的推理循环开始，到脚本检测到结束 Enter 为止，包含观测、推理和执行等待；不包含停止机器人等待、结果填写及手动复位。
- 保存每次尝试的原始 CSV 和 JSONL，核对 trace 的实际执行步数与本组设置一致；不要把预测 horizon 误当作执行步数。固定 40% 恰好等于 selector 的最短长度，但 selector 是动态执行，两者单独比较。

同名日志会追加历史记录。`--episodes 20` 表示本次新增 20 次尝试，不是自动补足到 20 次；补跑时先按上述口径核对已计入次数，再把 `--episodes` 改为剩余次数（例如已计入 8 次则使用 12）。保持原组步数、文件名及其他参数一致，避免重复整组运行。部署终端的汇总口径可能与本表不同，最终以原始 CSV 按上述规则统计。
