> 本文的 h3/h4、H10 和 spatial rule 命令为历史实验。当前 H20/H50 配对模型部署见 [deploy_selector.md](deploy_selector.md)。
> 2026-09-16 标签目录整理后，已迁移标签从 `data/datasets/selector/` 读取，可视化仍输出到 `data/analysis/`。

chunk selector标数据：称作spatial rule。
按照训练集任务轨迹，按照TCP的3D位置轨迹划分不同的任务阶段。
Threading real划分两段任务：第一个是非精细区，第二个是精细区。目前使用前50%后50%
maze任务是先精细区后非精细区。比较好的是前30%后70%。精细区的空间范围较小，非精细区的空间范围较大。

标注数据使用soft label。给予可能性。
精细区标h，非精细区标H。
两个任务当前统一使用 h=4、H=10。

标注好数据以后，我需要可视化chunk分布，以及节点对应的视频帧。


用于预测的神经网络，直接提取arp的视觉编码器MVT输出的特征。这里是点云。
使用一个transformer预测chunk。输出chunk可以是h~H之间的整数。

## 当前实现（spatial_rule_v1）

百分比默认按 TCP 3D **累计路程**计算，可通过 `--progress-mode frames` 改为帧数进度。
当前默认 `h=4`、`H=10`；两个任务部署输出均为 4–10 步。
Threading 使用 50% 节点、Maze 使用 30% 节点。已有权重是在 h=3 下训练的精细区概率模型；
当前部署将概率映射改为 `c = 4*p_fine + 10*(1-p_fine)`，取 `floor(c+0.5)`。
概率标签不依赖 h，因此无需重训。h4 部署快照及命令见 [deploy_selector.md](deploy_selector.md)。
60%、70%、80% 的结果保留用于历史对照。

当前 spatial rule 用球形区域描述精细区，与已有 endpoint schedule 的空间定义一致：

1. 按 episode 划分训练和验证集（默认 seed=42，验证占 20%）。
2. 仅从训练轨迹取中心：Threading 取末端 TCP 位置的逐坐标中位数，Maze 取起始 TCP 位置的逐坐标中位数。
3. 每条训练轨迹在指定进度插值得到一个 3D 节点。节点到中心距离的中位数为边界半径 `r`。
4. 距离 `d` 和完整过渡带宽 `w`（默认 2 cm）决定精细区概率：
   `p_fine = clip(0.5 + (r - d) / w, 0, 1)`。
5. soft label 为 `[p_fine, 1-p_fine]`，对应 `[h,H]`。连续期望为
   `c = h*p_fine + H*(1-p_fine)`；执行步数为 `floor(c+0.5)`。

因此分界位置处为 `[0.5,0.5]`。空间规则对所有 episode 相同：同一 TCP 位置得到同一标签，
不使用速度、全局分位排名或 fine 状态锁定。进度只用于拟合空间区域，并非逐帧按进度打标签。
球形边界是当前可解释的基线；若可视化发现轨迹绕行、多个任务阶段重叠或起点分布过宽，
应调整区域形状，而非假定一个球总能准确代表任务阶段。

**路程比例不等于帧数比例。** 当前 Threading 80% 路程分界得到的精细区平均约占每条轨迹
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

选取 70% 节点时，先找到累计路程达到 `总路程 × 0.7` 的位置。若该位置在两个观测之间，
用两点之间的线性插值确定空间节点；**导出截图时，则选进度最接近 70% 的实际帧**，
所以截图可能显示 70.01%，而不是恰好 70%。

同一条 Threading 轨迹（`episode_000000`）的实际例子如下，数据行号从 0 开始：

| 目标进度 | 最近实际帧的行号 | 该帧实际进度 |
| --- | --- | --- |
| 50% | 59 | 49.36% |
| 60% | 76 | 60.10% |
| 70% | 87 | 70.01% |
| 80% | 102 | 80.20% |

这些百分比节点用于估计精细区的空间范围；最终标签按 TCP 到精细区中心的距离生成。
因此，“70% 路程节点”不保证正好是进入精细区的那一帧，也不保证精细区占最后 30% 的帧。

## 标注与可视化

以下命令从 `/home/huiyuan/teleoperation` 运行，使用包含 NumPy、SciPy、PyArrow、
HDF5、OpenCV、Matplotlib、PyTorch 和 ARP 依赖的环境（本机为 conda `arp`）。
输出目录必须不存在，脚本不会覆盖已有结果。

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

两个任务共用 `threading_real/scripts/chunk_selector/` 的 spatial 标注、特征缓存与训练工具。
点云经过已训练并冻结的 ARP MVT 编码器，直接取其输出的空间 token；不向 selector 输入
TCP、速度、进度或动作。TCP 只用于离线构造监督。

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

训练检测到 `label_source=spatial_rule` 后自动启用概率交叉熵和 `selection_mode=expected`，
并沿用拟合区域时的训练/验证 episode 划分。神经网络输出两个概率，最终执行步数可以是
`h…10` 的任意整数，不局限于 h 和 10。以验证集 soft-label loss 选择 checkpoint，
同时记录连续 chunk MAE。更好的标签可学性与真实任务成功率仍需训练和实验验证。

## 推理接入

Threading Cartesian runner 的 `--chunk-selector` 已支持 MVT ARP；Maze runner 继续使用
`--selector`。二者都复用同一次 MVT 编码，动作模型生成完整预测，selector 决定执行前缀。
Threading MVT 当前使用 `--prediction-mode full_then_truncate`，不可同时指定
`--execution-schedule`。原始预测经过 runner 现有检查和裁剪后再执行所选前缀。

当前可用的模型配对、完整路径和部署命令统一见 [deploy_selector.md](deploy_selector.md)。
`selector_eval_20260914_142324` 保留 H10 历史模型与评估记录；使用时须配对原 H10 编码器。

## 直接导出 PNG（无需 HTML）

新增 `threading_real/scripts/chunk_selector/export_spatial_frames.py`。每个节点及相邻帧
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

已启动两个独立的“特征提取 → 完整性检查 → selector 训练”后台任务。
Threading 使用 50% 节点（64 条训练 / 16 条验证）；Maze 使用 30% 节点
（39 条训练 / 10 条验证）。两者均为 h=3、H=10、2 cm 过渡带、seed=42。

冻结 MVT 编码器，保留全部 1,800×128 token；提取 batch size=2，训练 batch size=4。
Selector 使用 2 层 Transformer、隐藏维度 256、4 个注意力头、前馈维度 1024、dropout=0.1，
AdamW 学习率 1e-4，最多 100 个 epoch，验证损失连续 15 个 epoch 不改善则早停。
使用概率交叉熵监督和期望取整输出，按最低验证损失保存模型。

- **threading**：[运行配置](../threading_real/outputs/selector_spatial_h3_p50_20260914_135535/run_config.json)、[状态](../threading_real/outputs/selector_spatial_h3_p50_20260914_135535/status.json)、[特征提取日志](../threading_real/outputs/selector_spatial_h3_p50_20260914_135535/extract.log)。训练开始后同目录生成 `train.log`，最佳模型保存为 `chunk_selector.safetensors`。
- **maze**：[运行配置](../maze_real/outputs/selector_spatial_h3_p30_20260914_135535/run_config.json)、[状态](../maze_real/outputs/selector_spatial_h3_p30_20260914_135535/status.json)、[特征提取日志](../maze_real/outputs/selector_spatial_h3_p30_20260914_135535/extract.log)。训练开始后同目录生成 `train.log`，最佳模型保存为 `chunk_selector.safetensors`。

每个 `run_config.json` 保存准确的 checkpoint 路径及 SHA-256、标注规则、数据划分和完整命令。
`status.json` 区分 `extract_running`、`train_running`、`complete`、`failed`，特征未提取完成时
不会提前启动训练，也不会将不完整缓存用于训练。后台任务不依赖聊天窗口保持打开。
