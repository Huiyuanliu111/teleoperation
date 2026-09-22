# Threading π0.5 OpenPI 部署：服务器 / 本机，no selector

> 2026-09-21：v4 已按用户要求删除，本机和服务器的2000步权重及对应配置已移除。下文引用v4的部署命令和验证结果仅为历史记录，不能直接启动；需先配置新的checkpoint。

本页包含服务器推理和本机推理两种完整流程。当前网络条件下推荐服务器推理：100组配对测试平均163 ms，本机4060 Ti平均287 ms。两种方式的相机采集和follower控制都在本机。

本机推理方式与 [LeRobot π0.5 本机部署](../threading_real/pi05/deployment/LOCAL_DEPLOYMENT.md) 一致：**权重位于本机，RTX 4060 Ti 直接推理，本机连接 cam1/cam3 相机及机器人控制服务**。

## 模型与运行环境

| 项目 | 设置 |
| --- | --- |
| 本机权重 | `threading_real/pi05_openpi/checkpoints/pi05_threading_lora/threading_lora_tcp6_30hz_h50_best_v4/2000` |
| 模型 | OpenPI π0.5 LoRA 配置，2000 步最佳权重，val loss `0.0295577834` |
| 本机环境 | `threading_real/pi05_openpi/.venv-deploy/bin/python` |
| 输入 | cam3/front + cam1/side 完整 RGB；9 维 TCP 状态，不含夹爪 |
| 预测 | 完整 50×6 动作，30 Hz 标签，10 次 flow 去噪 |
| 默认执行 | no selector，每次固定执行完整 **50 步**，随后重新观察推理 |
| 夹爪 | 每轮开始前夹取，随后保持闭合 |

权重和归一化统计已复制到本机，约 6.3 GB；未复制优化器状态。独立 OpenPI 环境使用 JAX 0.5.3，同时只读复用已有 LeRobot 部署环境中的部分依赖（包括 Pinocchio 和 RealSense）；不要删除 `pi05/.venv-deploy`。原 LeRobot 环境未修改。

部署位置与 LeRobot launcher 一致，但执行长度改为固定 50 步，避免每次重新推理只执行一个微小动作。可显式选择更短前缀。OpenPI 保持训练对应的 30 Hz，不沿用旧 LeRobot 数据的 15 Hz。

## 服务器推理部署（推荐）

三个终端依次启动。终端一加载服务器权重，终端二保持SSH转发，终端三在相机连接的本机上运行。

```text
本机 cam1/cam3 + follower 实测关节状态
          ↓ 本机 FK、完整 RGB letterbox224
服务器 10.157.174.249：OpenPI 2000步权重 → 50×6动作
          ↓ 返回本机
本机：检查动作、积分、限幅、TrackC UDP
          ↓
follower 10.157.175.22：执行机器人控制
```

服务器不连接相机，也不直接控制机器人。网络传输的是两张224×224 RGB、9维状态和任务文本，图像约301 KB/请求，返回动作约1.2 KB。图像使用原始uint8无损传输，没有JPEG压缩、ROI、新增平滑或动作放大。

每次预测50步，默认固定执行50步，动作时间30 Hz；本机TrackC发送480 Hz。仍只等待发送完成，目标跟踪误差仅记录。夹取力70 N，使用当前两阶段放置/夹取流程。

### 终端一：启动服务器模型

```bash
ssh huiyuan@10.157.174.249
```

在服务器终端运行（已检查GPU2空闲；启动前再次确认占用）：

```bash
cd /home/huiyuan/threading_real/pi05_openpi
CUDA_VISIBLE_DEVICES=2 JAX_PLATFORMS=cuda \
XLA_PYTHON_CLIENT_MEM_FRACTION=0.8 OMP_NUM_THREADS=2 \
PYTHONPATH=/home/huiyuan/threading_real/pi05_openpi/vendor/lerobot \
/home/huiyuan/threading_real/pi05_openpi/.venv/bin/python -u \
  /home/huiyuan/threading_real/pi05_openpi/serve_policy.py \
  --checkpoint /home/huiyuan/threading_real/pi05_openpi/checkpoints/pi05_threading_lora/threading_lora_tcp6_30hz_h50_best_v4/2000 \
  --run-config /home/huiyuan/threading_real/pi05_openpi/checkpoints/pi05_threading_lora/threading_lora_tcp6_30hz_h50_best_v4.json \
  --host 127.0.0.1 --port 8000
```

等到 `server listening on 127.0.0.1:8000`。

### 终端二：本机转发

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 18000:127.0.0.1:8000 huiyuan@10.157.174.249
```

### 终端三：本机相机和机器人部署

```bash
cd /home/huiyuan/teleoperation
bash threading_real/pi05_openpi/run_deploy_remote.sh \
  --prepare-initial-grasp --grasp-before-inference \
  --initial-grasp-width 0.02 --gripper-force 70 \
  --execute-steps 50 --no-sync-require-target \
  --execute --confirm-real-robot
```

本机不加载JAX模型、不占GPU推理显存。位置参数是服务器权重路径，客户端会与服务器元数据核对版本。

20轮实验可使用：

```bash
cd /home/huiyuan/teleoperation
bash threading_real/pi05_openpi/run_deploy_remote.sh \
  --episodes 20 --max-cycles 0 \
  --prepare-initial-grasp --grasp-before-inference \
  --initial-grasp-width 0.02 --gripper-force 70 \
  --execute-steps 50 --no-sync-require-target \
  --trace-output /home/huiyuan/teleoperation/data/analysis/pi05_openpi_remote_eval/logs/no_selector_h50.jsonl \
  --results-csv /home/huiyuan/teleoperation/data/analysis/pi05_openpi_remote_eval/logs/no_selector_h50.csv \
  --execute --confirm-real-robot
```

### 网络与时间

连接机器人前，客户端会发送一份合成观测预热服务器，丢弃其预测，预热最长等待120秒。正式每次推理默认5秒超时（`--inference-timeout`），没有自动重发；超时关闭连接并退出，runner执行原有停止清理流程，不接受迟到的动作。

每轮打印 `[remote] roundtrip=...s actions=50x6`，包含图像上传、推理和返回。仍为同步顺序：观察→推理→执行50步→重新观察，不进行异步动作拼接。不能把30 Hz动作频率解释为30 Hz重新推理。

本机和远程推理入口不能同时占用机器人或相机。本次准备仅用记录数据测试网络推理，没有启动相机或机器人。验证记录：`threading_real/pi05_openpi/outputs/remote_deployment_smoke.json`。

### 延迟对照（2026-09-21）

相同100个记录观测、相同2000步权重、H50、10次去噪；两侧预热后交替测量。

| 方式 | 平均 | 中位数 | P95 |
| --- | ---: | ---: | ---: |
| 本机 RTX 4060 Ti | 287 ms | 287 ms | 295 ms |
| 服务器 A40，含SSH传输往返 | 163 ms | 163 ms | 168 ms |

服务器路线100/100次更快，平均延迟减少约43%；服务器内部推理157 ms，其余传输及序列化等开销约6 ms。不包含相机采集、FK和机器人执行。网络或GPU负载变化可能改变结果。详见 [完整配对测试报告](../threading_real/pi05_openpi/outputs/latency_comparison.md)。

## 本机推理部署

在相机连接的本机终端运行。以下命令默认不发送模型动作，但会连接机器人状态服务、初始化相机：

```bash
cd /home/huiyuan/teleoperation
bash threading_real/pi05_openpi/run_deploy.sh --max-cycles 20
```

### no selector：固定执行 50 步

```bash
cd /home/huiyuan/teleoperation
mkdir -p /home/huiyuan/teleoperation/data/analysis/pi05_openpi_eval_20260921/logs

bash /home/huiyuan/teleoperation/threading_real/pi05_openpi/run_deploy.sh \
  --execute-steps 50 \
  --episodes 20 --max-cycles 0 \
  --grasp-before-inference --initial-grasp-width 0.02 \
  --trace-output /home/huiyuan/teleoperation/data/analysis/pi05_openpi_eval_20260921/logs/threading_openpi_h50_exec50_no_selector.jsonl \
  --results-csv /home/huiyuan/teleoperation/data/analysis/pi05_openpi_eval_20260921/logs/threading_openpi_h50_exec50_no_selector.csv \
  --execute --confirm-real-robot
```

每轮先将机械臂放到抓取位置并退出 guide，按 Enter 准备。OpenPI 默认启用 `--prepare-initial-grasp`：先完全张开夹爪并等待完成，再提示将物体放到两指之间。手离开夹爪后再次按 Enter，执行夹取；只有反馈 HOLDING 才开始相机预热和正式观测推理。运行中按 Enter 结束当前轮。runner 停止后再手动进入 guide 复位，按提示记录结果。本入口不自动切换 guide，不自动移动到训练起始位姿。

相机序列号、机器人地址、夹取设置沿用现有 Threading Cartesian runner。不得与其他占用相机或控制机器人的部署进程同时运行。


## 实际处理与时间含义

- 模型在本机 CUDA GPU 上运行；launcher 中的 `--device cpu` 仅用于旧 runner 的观测包装，不表示模型在 CPU 上推理。
- 不新增平滑、ROI、图像增强或候选筛选。完整 640×480 RGB 缩放为 224×168，上下各补 28 行黑边，与训练处理一致。
- 使用相同 Panda URDF 做 FK，状态为 xyz 和旋转矩阵前两列，共 9 维。
- 输出为基座坐标系平移增量（米）和左乘旋转增量（旋转向量，弧度）。模型输出 6 维；适配器给旧控制接口补固定零夹爪通道，并禁用逐 chunk 的夹爪命令。每轮开始前的夹取仍执行。
- 复用现有 TrackC 路径插值，设置 `--stream-hz 480`，每个 30 Hz 动作段发送 16 个采样点，避免默认 500 Hz 除以 30 取整造成时长偏差。
- 保留既有动作限幅：首步平移 0.040 m，后续 0.025 m；首步旋转 0.40 rad，后续 0.25 rad。执行前对所选动作检查：原始平移 >0.15 m 或旋转 >1.50 rad 时拒绝执行。限幅可能改变实际动作，trace 保留对应统计。
- 同步执行，`--sync-timeout 5.0`。30 Hz 是动作轨迹的时间间隔；推理与同步等待另计，**不是每秒重规划 30 次**。执行 1 步时，每轮仍承担完整 H50 推理耗时。
- 每次推理使用一个随机 flow 噪声样本；没有对多个预测取平均。首次推理需要 JAX 编译。

## 验证

准备过程只使用记录数据做本机 GPU 推理与接口测试，不连接相机、不初始化机器人、不发送动作。本机验证结果保存在 `threading_real/pi05_openpi/outputs/local_deployment_smoke.json`。这不代表已完成实机闭环测试。

本机实测：模型加载约 10.1 秒，首次含编译推理约 21.7 秒，后续两次 H50 推理约 0.260 / 0.258 秒。结果仅覆盖记录观测的离线推理，不含相机采集、控制器与同步等待。



## 2026-09-21 执行与跟踪记录

OpenPI 默认固定执行完整 H50，使用 `--no-sync-require-target`。同步等待只要求本段 UDP 轨迹发送完成，再记录实际跟踪误差；不要求机械臂达到目标容差，不因稳态跟踪误差中断运行。发送过程未完成或机器人状态异常仍会报错。

每轮积分前重新读取机器人状态；输出 `[tracking] steps=50 target_delta=...mm actual_delta=...mm`。`sync_target=False` 仅表示记录的误差超出参考容差，不阻断下一轮。动作不放大、刚度不提高，保持夹取流程和70 N设置。

如果之前命令中显式添加了 `--sync-require-target`，请删除或改为 `--no-sync-require-target`，否则会覆盖 launcher 默认设置。
