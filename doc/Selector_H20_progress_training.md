# Threading H20 selector 重训（2026-09-16）

本次修复旧 H10 selector 与新 H20 MVT 编码器不兼容的问题。冻结 H20 动作模型，重新提取全部
视觉 token 后从头训练 selector。本记录仅涵盖 Threading；后续 Maze H50 模型见
[部署文档](deploy_selector.md)。

- 动作模型：`training_runs/planarp_chunks_20260914_165545/threading_planarp_chunk20/checkpoints/epoch=0008-val_loss=10.981.ckpt`，使用 `model` 权重。
- checkpoint SHA-256：`fddcfe4633db7c28799da7138e6d70238c3110da3770043a6a9b19b68da250c3`。
- 标签：每条完整轨迹按 TCP 累计路程归一化；0–45% 为粗区，45–55% 线性过渡，55–100% 为细区。
- 候选 `[8,20]`，软概率交叉熵；执行长度为期望值四舍五入。完整路程仅用于离线监督，不作为在线模型输入。
- 80 条轨迹、11,273 帧；沿用 seed=42 的 64/16 条训练/验证划分。
- 全部 1,800×128 MVT token，无池化。特征提取 batch size=2。
- Selector：2 层 Transformer，d_model=256，4 个头，FFN=1024，dropout=0.1。
- AdamW，学习率 1e-4，weight decay=1e-4，batch size=4，最多 100 epoch，patience=15。
- 按验证 soft-label loss 保存最佳权重；W&B online 记录训练/验证 loss、accuracy、macro F1、chunk MAE。
- W&B 项目 `huiyuan_tac/adaptive_chunk`，run 名 `threading_selector_H20_h8_progress_20260916_134300`。

2026-09-16 13:49（Europe/Berlin）：全部 11,273 帧特征通过完整性检查，训练已启动。
W&B online：[q03rscg8](https://wandb.ai/huiyuan_tac/adaptive_chunk/runs/q03rscg8)。

2026-09-16 14:07 起按用户要求停止：完成 11 个 epoch，在第 12 个 epoch 中途停止，
保留最佳 epoch 11。验证 loss=0.051582，accuracy=0.992484，macro F1=0.992193，
chunk MAE 约 0.142 步。停止后的最佳模型完整验证已完成，W&B 同一 run 已正常收尾。
`status.json` 标记 `stopped_by_user`；原 SIGINT 中断记录保留在 `interruption_status.json`。

后台任务顺序为：特征提取 → 完整性与 checkpoint 哈希核验 → 训练 → 最佳模型验证与路程曲线。
任一步失败会写入 `failed` 状态，不会继续下一步。关闭聊天不会停止后台进程。

运行文件：

- [完整配置及命令](../training_runs/threading_selector_H20_h8_progress_20260916_134300/run_config.json)
- [当前状态](../training_runs/threading_selector_H20_h8_progress_20260916_134300/status.json)
- [特征提取日志](../training_runs/threading_selector_H20_h8_progress_20260916_134300/extract.log)
- [训练日志](../training_runs/threading_selector_H20_h8_progress_20260916_134300/train.log)（训练开始后生成）
- [W&B run 信息](../training_runs/threading_selector_H20_h8_progress_20260916_134300/model/wandb_run.json)（online 初始化后生成）
- [训练历史](../training_runs/threading_selector_H20_h8_progress_20260916_134300/model/training_history.json)（逐 epoch 更新）

特征保存为 `data/features/threading_H20_h8_progress_20260916_134300.h5`。
最佳权重保存到运行目录的 `model/chunk_selector.safetensors`，并带匹配编码器的哈希元数据。
训练完成后生成 `model/validation_metrics.json`、`model/validation_predictions.csv` 和
`model/validation_chunks_by_arc_length.png`。这些是离线验证，不代表真机成功率或切换效果。
