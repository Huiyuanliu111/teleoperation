> 按 2026-09-18 的代码和已保存模型配置核对。当前默认部署为 Threading H20 / h8 与 Maze H10 / h4，完整命令见 [deploy_selector.md](deploy_selector.md)。
> 已迁移的历史空间标签位于 `data/datasets/selector/`；当前 Threading 路程标签保存在对应 `training_runs/` 运行目录，可视化和评估路径见下文。

Chunk selector 使用 TCP 轨迹构造离线 soft label，精细阶段对应短 chunk `h`，非精细阶段对应长 chunk `H`。
当前有两种标签规则，不能统一称作 spatial rule：

| 项目 | Threading 当前部署 | Maze 当前部署 |
| --- | --- | --- |
| 动作模型 horizon | 20 | 10 |
| 执行范围 | 8–20 步 | 4–10 步 |
| 标签来源 | `progress_rule`，逐帧按累计路程进度标注 | `spatial_rule`，按 TCP 到球心的距离标注 |
| 阶段顺序 | 先粗后细，50% 路程为过渡中心 | 先细后粗，用 30% 路程节点拟合球形边界 |
| 过渡带 | 45–55% 路程，即完整宽度 0.1 | 球形边界内外完整宽度 2 cm |
| 权重来源 | 配对 H20 MVT 编码器重新提取特征并重训 | 历史 h3/H10 概率权重重映射为 h4/H10 |

两个任务均以点云为输入，提取冻结的 ARP MVT 编码器输出的空间 token，交给 Transformer
预测两个概率。令精细区概率为 `p_fine`，连续期望为 `c = h*p_fine + H*(1-p_fine)`，
执行步数为 `floor(c+0.5)`，可以取 `h…H` 的任意整数。TCP、速度、进度和动作不作为 selector 的在线输入。

## 当前 Threading：累计路程标签（arc_length_progress_v1）

实现见 [`progress_rule.py`](../threading_real/chunk_selector/progress_rule.py) 和
[`label_progress.py`](../threading_real/scripts/chunk_selector/label_progress.py)。
每条完整轨迹独立计算 TCP 3D 累计路程进度 `s`，当前模型使用：

```text
p_fine = clip(0.5 + (s - 0.5) / 0.1, 0, 1)
soft_label = [p_fine, 1-p_fine]
c = 8*p_fine + 20*(1-p_fine)
execution_steps = floor(c + 0.5)
```

因此 0–45% 为粗区，45–55% 线性过渡，55–100% 为细区。这里直接按进度生成标签，
不拟合球心或半径；同一 TCP 位置可能因所在轨迹或进度不同而得到不同标签。
`progress_rule` 仅支持累计路程，不提供帧数进度选项。完整轨迹仅用于离线监督，在线由视觉预测概率。

当前使用的文件：

- [标签与规则](../training_runs/threading_selector_H20_h8_progress_20260916_134300/labels/summary.json)：80 条轨迹、11,273 帧，64 条训练 / 16 条验证，seed=42。
- [模型配置](../training_runs/threading_selector_H20_h8_progress_20260916_134300/model/chunk_selector_config.json)：候选 `[8,20]`，`label_source=progress_rule`，匹配 H20 epoch 8 编码器及其 SHA-256。
- [完整提取、训练与验证命令](../training_runs/threading_selector_H20_h8_progress_20260916_134300/run_config.json)。
- [运行状态](../training_runs/threading_selector_H20_h8_progress_20260916_134300/status.json)：按用户要求停止，完成 11 个 epoch，最佳 epoch 11，停止后验证已完成。

训练细节见 [Selector_H20_progress_training.md](Selector_H20_progress_training.md)。模型输入保留全部
1,800×128 token，无池化。更换动作模型的 MVT 编码器后，不能仅修改候选长度就认为旧 selector 仍然匹配；
当前 Threading 已使用配对编码器重新提取特征并重训。

## 当前 Maze 与历史 Threading：空间标签（spatial_rule_v1）

实现见 [`spatial_rule.py`](../threading_real/chunk_selector/spatial_rule.py)。`label_spatial.py`
默认 `h=4`、固定 `H=10`，允许 h 为 3、4、5；这不是两个任务当前部署的统一配置。
百分比默认按 TCP 3D 累计路程计算，可通过 `--progress-mode frames` 改为帧数进度。
未指定 `--split-progress` 时，脚本默认 Threading 0.8、Maze 0.3；历史 Threading p50 实验显式传入 0.5。

Maze 当前使用 [maze_h4 模型配置](../data/analysis/selector_eval_20260914_142324/models/maze_h4/chunk_selector_config.json)，
由历史 h3/H10 概率权重重映射为 h4/H10，执行 `floor(4*p_fine + 10*(1-p_fine) + 0.5)`。
仅在标签规则和视觉编码器保持一致、只调整输出映射时，才可利用概率标签不依赖 h 的性质而无需重训。
Threading 的 p50、p60、p70、p80 空间规则结果均作为历史对照。

spatial rule 用球形区域描述精细区：

1. 按 episode 划分训练和验证集（默认 seed=42，验证占 20%）。
2. 仅从训练轨迹取中心：Threading 取末端 TCP 位置的逐坐标中位数，Maze 取起始 TCP 位置的逐坐标中位数。
3. 每条训练轨迹在指定进度插值得到一个 3D 节点。节点到中心距离的中位数为边界半径 `r`。
4. 距离 `d` 和完整过渡带宽 `w`（默认 2 cm）决定精细区概率：
   `p_fine = clip(0.5 + (r - d) / w, 0, 1)`。
5. soft label 为 `[p_fine, 1-p_fine]`，对应 `[h,H]`。连续期望为
   `c = h*p_fine + H*(1-p_fine)`；执行步数为 `floor(c+0.5)`。

因此球形分界位置处为 `[0.5,0.5]`。对于同一套已拟合的空间规则，所有 episode 共用该规则：同一 TCP 位置得到同一标签，
不使用速度、全局分位排名或 fine 状态锁定。进度只用于拟合空间区域，并非逐帧按进度打标签。
上述性质仅适用于 spatial rule，不适用于当前 Threading 的 progress rule。球形边界是空间标注基线；若可视化发现轨迹绕行、多个任务阶段重叠或起点分布过宽，
应调整区域形状，而非假定一个球总能准确代表任务阶段。

**路程比例不等于帧数比例。** 历史 Threading spatial rule 的 80% 路程分界得到的精细区平均约占每条轨迹
32% 的帧。报告同时标出进度节点和实际空间边界的穿越节点，可能不在同一帧。

## 归一化进度：50%、60%、70% 是怎样计算的

这里的百分比表示：**TCP 已经走过的路程，占这条轨迹总路程的比例。**
每条轨迹单独计算，起点为 0%，终点为 100%。

先读取每一帧 TCP 的三维位置 `(x, y, z)`，计算相邻两帧之间的距离：

```text
第 k 段距离 = sqrt(
    (x[k] - x[k-1])²
  + (y[k] - y[k-1])²
  + (z[k] - z[k-1])²
)
```

再把这些距离累加，并除以整条轨迹的总路程：

```text
累计路程[k] = 第 1 段距离 + ... + 第 k 段距离
总路程      = 所有相邻帧之间的距离之和
归一化进度[k] = 累计路程[k] / 总路程
百分比进度[k] = 归一化进度[k] × 100%
```

例如，一条轨迹总共走了 40 cm：走到 20 cm 时是 50%，走到 24 cm 时是 60%，
走到 28 cm 时是 70%。如果总路程为零，则无法这样归一化，程序会报错。

这里按沿途路程计算，**不是按帧数或时间计算**。走得慢会占用更多帧，但不一定增加更多
路程；位置不变的暂停不增加路程，折返则仍然增加路程。它也不是起点到当前位置的直线距离。
当前只计算 TCP 的平移，不计旋转和夹爪开合；位置噪声和采样疏密会影响计算结果。

对于 spatial rule，选取 70% 节点时，先找到累计路程达到 `总路程 × 0.7` 的位置。若该位置在两个观测之间，
用两点之间的线性插值确定空间节点；**导出截图时，则选进度最接近 70% 的实际帧**，
所以截图可能显示 70.01%，而不是恰好 70%。

同一条 Threading 轨迹（`episode_000000`）的实际例子如下，数据行号从 0 开始：

| 目标进度 | 最近实际帧的行号 | 该帧实际进度 |
| --- | --- | --- |
| 50% | 59 | 49.36% |
| 60% | 76 | 60.10% |
| 70% | 87 | 70.01% |
| 80% | 102 | 80.20% |

在 spatial rule 中，这些百分比节点用于估计精细区的空间范围；最终标签按 TCP 到精细区中心的距离生成。
因此，“70% 路程节点”不保证正好是进入精细区的那一帧，也不保证精细区占最后 30% 的帧。

## 当前 Threading 路程标签的生成与可视化

以下命令从仓库根目录、在 conda `arp` 环境中运行，输出到新的复现目录；不会修改当前部署模型：

```bash
python threading_real/scripts/chunk_selector/label_progress.py \
  data/datasets/threading_combined_80_mvt_cam1_7p5hz.h5 \
  --task threading --h 8 --H 20 --split-progress 0.5 --transition-width 0.1 \
  --previous-labels data/datasets/selector/threading_spatial_h3_p50 \
  --seed 42 --val-ratio 0.2 \
  --output data/datasets/selector/threading_progress_h8_H20_p50_repro
```

`--previous-labels` 用于核验旧标签的 episode 划分与逐帧 TCP，并记录标签变化，不决定新标签概率。
输出目录必须不存在。脚本生成 `labels.parquet`、`summary.json` 和 `labels_by_arc_length.png`。
已有训练结果的验证曲线为
[validation_chunks_by_arc_length.png](../training_runs/threading_selector_H20_h8_progress_20260916_134300/model/validation_chunks_by_arc_length.png)，
验证指标和逐帧预测也保存在同一模型目录。`evaluate_progress.py` 的实际调用见上面的运行配置。
`visualize_spatial.py` 依赖球形规则字段，不能直接用于 progress_rule 标签。

## 空间标签的标注与可视化（历史复现实例）

以下命令从 `/home/huiyuan/teleoperation` 运行，使用包含 NumPy、SciPy、PyArrow、
HDF5、OpenCV、Matplotlib、PyTorch 和 ARP 依赖的环境（本机为 conda `arp`）。
输出目录必须不存在，脚本不会覆盖已有结果。以下 h4 空间标签示例不复现当前 Threading 模型，
也不代表 Maze 当前快照曾以 h4 重新训练。

```bash
python threading_real/scripts/chunk_selector/label_spatial.py \
  data/datasets/threading_combined_80_mvt_cam1_7p5hz.h5 \
  --task threading --h 4 --split-progress 0.5 \
  --output data/datasets/selector/threading_spatial_h4_p50

python threading_real/scripts/chunk_selector/label_spatial.py \
  data/datasets/maze_train49_mvt_7p5hz.h5 \
  --task maze --h 4 --split-progress 0.3 \
  --output data/datasets/selector/maze_spatial_h4_p30

python threading_real/scripts/chunk_selector/visualize_spatial.py \
  data/datasets/selector/maze_spatial_h4_p30 --output data/analysis/maze_spatial_h4_p30/report
```

标注输出 `labels.parquet` 和 `summary.json`，保存完整规则、训练/验证 episode 清单、
3D 节点、概率、连续 chunk、整数执行步数及每条轨迹的边界穿越帧。

可视化输出 `report/index.html`、PNG 分布图、每条轨迹的 3D/进度图、节点前后的视频帧，
以及 `video_frames.json`。视频依据 HDF5 的 `camera_frame_index` 精确对应，不以
降采样后的行号直接代替视频帧号。默认包括进度节点前后各 3 行和所有空间穿越节点；
可通过 `--episodes episode_000000 ...` 选择 episode，通过 `--frame-offset` 调整前后范围。
用 `--raw-root` 指定搬迁后的原始视频根目录。视频缺失时仍生成图表并明确记录缺失原因。

旧报告及截图已清理，保留以下标签与规则：

- [Threading 80% 标签](../data/datasets/selector/threading_spatial_h3_p80/summary.json)：80 条轨迹、11,273 帧。
- [Threading 70% 标签](../data/datasets/selector/threading_spatial_h3_p70/summary.json)。
- [Maze 30% 标签](../data/datasets/selector/maze_spatial_h3_p30/summary.json)：49 条轨迹、3,437 帧。

需要时重新运行可视化命令。Threading 原始视频根目录 `data/threading_new` 当前不存在，
点云渲染可直接使用 HDF5，原视频截图仍需对应原始视频。

## ARP MVT 特征 → Transformer selector

两个任务共用 `threading_real/scripts/chunk_selector/` 的特征缓存与训练工具，
分别使用 `label_progress.py` 或 `label_spatial.py` 生成监督。
点云经过已训练并冻结的 ARP MVT 编码器，直接取其输出的空间 token；不向 selector 输入
TCP、速度、进度或动作。TCP 只用于离线构造监督。

以下是与上一节 h4 空间标签配套的历史训练示例。当前 Threading H20 的准确提取、训练命令见
[run_config.json](../training_runs/threading_selector_H20_h8_progress_20260916_134300/run_config.json)。

```bash
python threading_real/scripts/chunk_selector/extract_mvt_features.py \
  threading_real/outputs/threading_combined_80_mvt_cam1_planarp_v2/20260912_171342/checkpoints/latest.ckpt \
  data/datasets/selector/threading_spatial_h4_p50 \
  --dataset-path data/datasets/threading_combined_80_mvt_cam1_7p5hz.h5 \
  --output data/threading_spatial_h4_mvt_features.h5 \
  --device cuda:0 --batch-size 2

python threading_real/scripts/chunk_selector/extract_mvt_features.py \
  maze_real/outputs/maze_planarp_train49_7p5hz/checkpoints/epoch_0209.pt \
  data/datasets/selector/maze_spatial_h4_p30 \
  --dataset-path data/datasets/maze_train49_mvt_7p5hz.h5 \
  --output data/maze_spatial_h4_mvt_features.h5 \
  --device cuda:0 --batch-size 2

python threading_real/scripts/chunk_selector/train.py \
  data/threading_spatial_h4_mvt_features.h5 \
  --output-dir threading_real/outputs/selector_spatial_h4_p50 \
  --device cuda:0 --batch-size 4 --epochs 100

python threading_real/scripts/chunk_selector/train.py \
  data/maze_spatial_h4_mvt_features.h5 \
  --output-dir maze_real/outputs/selector_spatial_h4_p30 \
  --device cuda:0 --batch-size 4 --epochs 100
```

缓存默认保留全部 MVT token（420/14 时为两个虚拟视角各 30×30，共 1,800 个）。
如需减少显存和缓存体积，可在提取时显式添加 `--pool-grid 4`，将每个虚拟视角池化到
4×4，得到 32 个 token；该设置写入 checkpoint，部署使用相同处理。

训练检测到 `label_source` 为 `spatial_rule` 或 `progress_rule` 后自动启用概率交叉熵和
`selection_mode=expected`，并沿用标签中保存的训练/验证 episode 划分。神经网络输出两个概率，最终执行步数可以是
`h…H` 的任意整数，不局限于两个端点。以验证集 soft-label loss 选择 checkpoint，
同时记录连续 chunk MAE。更好的标签可学性与真实任务成功率仍需训练和实验验证。

## 推理接入

Threading Cartesian runner 的 `--chunk-selector` 已支持 MVT ARP；Maze runner 继续使用
`--selector`。二者都复用同一次 MVT 编码，动作模型生成完整预测，selector 决定执行前缀。
Threading MVT 当前使用 `--prediction-mode full_then_truncate`，不可同时指定
`--execution-schedule`。原始预测经过 runner 现有检查和裁剪后再执行所选前缀。

当前可用的模型配对、完整路径和部署命令统一见 [deploy_selector.md](deploy_selector.md)。
`selector_eval_20260914_142324` 保留 H10 模型与评估记录，其中 `models/maze_h4` 仍是当前
Maze 默认 selector，须配对原 H10 编码器。Threading 使用上述 H20 progress 模型。
AAC 是 runner 的另一种自适应执行长度方式，不加载 learned selector，不能与 selector 同时启用；
其部署命令和评估另见部署文档。

## 直接导出 PNG（无需 HTML）

`threading_real/scripts/chunk_selector/export_spatial_frames.py` 用于导出节点帧。以下为空间标签示例。每个节点及相邻帧
单独保存为 PNG，并按 episode 输出 `episode_XXXXXX_overview.png` 拼图。
`frames.json` 记录来源、数据行号、原视频帧号和节点类型。

数据目录整理后可通过 `--dataset` 和 `--raw-root` 覆盖历史元数据中的旧路径：

```bash
python threading_real/scripts/chunk_selector/export_spatial_frames.py \
  data/datasets/selector/maze_spatial_h3_p30 \
  --dataset data/datasets/maze_train49_mvt_7p5hz.h5 \
  --raw-root data/raw/maze_data_train49 \
  --source-kind video --output data/analysis/maze_spatial_h3_p30/png_frames

python threading_real/scripts/chunk_selector/export_spatial_frames.py \
  data/datasets/selector/threading_spatial_h3_p80 \
  --dataset data/datasets/threading_combined_80_mvt_cam1_7p5hz.h5 \
  --source-kind pointcloud --device cuda:0 \
  --output data/analysis/threading_spatial_h3_p80/pointcloud_png
```

以上输出曾生成，现已清理；重新运行可恢复相应输出（原视频截图需原始视频可用）。Threading 的 HDF5 可直接用于标注、特征提取和训练，但其
`colors[T,N,3]` 是点的颜色，不是原始 RGB 图像；点云 PNG 使用 MVT top/left 虚拟视角，
图片中明确标注来源，不能当作原视频截图。

## 历史 h3 后台训练（2026-09-14）

两个独立的“特征提取 → 完整性检查 → selector 训练”后台任务均已完成，
各自 `status.json` 的 `state` 为 `complete`；本节记录历史配置。
Threading 使用 50% 节点（64 条训练 / 16 条验证）；Maze 使用 30% 节点
（39 条训练 / 10 条验证）。两者均为 h=3、H=10、2 cm 过渡带、seed=42。

冻结 MVT 编码器，保留全部 1,800×128 token；提取 batch size=2，训练 batch size=4。
Selector 使用 2 层 Transformer、隐藏维度 256、4 个注意力头、前馈维度 1024、dropout=0.1，
AdamW 学习率 1e-4，最多 100 个 epoch，验证损失连续 15 个 epoch 不改善则早停。
使用概率交叉熵监督和期望取整输出，按最低验证损失保存模型。

- **threading**：[运行配置](../threading_real/outputs/selector_spatial_h3_p50_20260914_135535/run_config.json)、[状态](../threading_real/outputs/selector_spatial_h3_p50_20260914_135535/status.json)、[特征提取日志](../threading_real/outputs/selector_spatial_h3_p50_20260914_135535/extract.log)。同目录已生成 `train.log`，最佳模型保存为 `chunk_selector.safetensors`。
- **maze**：[运行配置](../maze_real/outputs/selector_spatial_h3_p30_20260914_135535/run_config.json)、[状态](../maze_real/outputs/selector_spatial_h3_p30_20260914_135535/status.json)、[特征提取日志](../maze_real/outputs/selector_spatial_h3_p30_20260914_135535/extract.log)。同目录已生成 `train.log`，最佳模型保存为 `chunk_selector.safetensors`。

每个 `run_config.json` 保存准确的 checkpoint 路径及 SHA-256、标注规则、数据划分和完整命令。
`status.json` 区分 `extract_running`、`train_running`、`complete`、`failed`，特征未提取完成时
不会提前启动训练，也不会将不完整缓存用于训练。后台任务不依赖聊天窗口保持打开。
