# OpenPI 服务器推理、本机相机、follower 控制

> 2026-09-22 清理更新：已按用户要求删除服务器 threading real 的全部实验结果，包括 v11/2000、v10/500 和对应配置、归一化统计；训练及 v11 推理服务均已停止。下文部署命令为历史记录，当前不能直接启动，需先配置新的 checkpoint。

这是独立的远程推理入口，本机推理的 `run_deploy.sh` 保留。

```text
本机 cam1/cam3 + follower 实测关节状态
          ↓ 本机 FK、完整 RGB letterbox224
服务器 10.157.174.249：OpenPI v11 2000步权重 → 10×6动作
          ↓ 返回本机
本机：检查动作、积分、限幅、TrackC UDP
          ↓
follower 10.157.175.22：执行机器人控制
```

服务器不连接相机，也不直接控制机器人。网络传输的是两张224×224 RGB、9维状态和任务文本，图像约301 KB/请求，返回动作约240 B（float32，不含协议开销）。图像使用原始uint8无损传输，没有JPEG压缩、ROI、新增平滑或动作放大。

每次预测10步，默认固定执行10步（约0.333秒），动作时间30 Hz；本机TrackC发送480 Hz。仍只等待发送完成，目标跟踪误差仅记录。夹取力70 N，使用当前两阶段放置/夹取流程。

## 终端一：启动服务器模型

```bash
ssh huiyuan@10.157.174.249
```

在服务器终端运行（已检查GPU2空闲；启动前再次确认占用）：

```bash
cd /home/huiyuan/threading_real/pi05_openpi
CUDA_VISIBLE_DEVICES=2 JAX_PLATFORMS=cuda \
XLA_PYTHON_CLIENT_MEM_FRACTION=0.8 OMP_NUM_THREADS=4 \
PYTHONPATH=/home/huiyuan/threading_real/pi05_openpi/vendor/lerobot \
/home/huiyuan/threading_real/pi05_openpi/.venv/bin/python -u \
  /home/huiyuan/threading_real/pi05_openpi/serve_policy.py \
  --checkpoint /home/huiyuan/threading_real/pi05_openpi/checkpoints/pi05_threading_vision_lora_action_full/threading_tcp6_30hz_h10_vision_lora_action_full_v11/2000 \
  --run-config /home/huiyuan/threading_real/pi05_openpi/checkpoints/pi05_threading_vision_lora_action_full/threading_tcp6_30hz_h10_vision_lora_action_full_v11.json \
  --host 127.0.0.1 --port 8000
```

等到 `server listening on 127.0.0.1:8000`。

## 终端二：本机转发

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 18000:127.0.0.1:8000 huiyuan@10.157.174.249
```

## 终端三：本机相机和机器人部署

```bash
cd /home/huiyuan/teleoperation
bash threading_real/pi05_openpi/run_deploy_remote.sh \
  --prepare-initial-grasp --grasp-before-inference \
  --initial-grasp-width 0.02 \
  --execute --confirm-real-robot
```

本机不加载JAX模型、不占GPU推理显存。`run_deploy_remote.sh` 已默认指定 v11/2000 的服务器权重路径及 `--execute-steps 10`，客户端会与服务器元数据核对版本和动作长度。服务端从 run-config 读取 horizon=10；不要使用仍写死 H50 的旧版部署代码。

20轮实验可使用：

```bash
cd /home/huiyuan/teleoperation
bash threading_real/pi05_openpi/run_deploy_remote.sh \
  --episodes 20 --max-cycles 0 \
  --prepare-initial-grasp --grasp-before-inference \
  --initial-grasp-width 0.02 \
  --trace-output /home/huiyuan/teleoperation/data/analysis/pi05_openpi_remote_eval/logs/v11_2000_h10.jsonl \
  --results-csv /home/huiyuan/teleoperation/data/analysis/pi05_openpi_remote_eval/logs/v11_2000_h10.csv \
  --execute --confirm-real-robot
```

## 网络与时间

连接机器人前，客户端会发送一份合成观测预热服务器，丢弃其预测，预热最长等待120秒。正式每次推理默认5秒超时（`--inference-timeout`），没有自动重发；超时关闭连接并退出，runner执行原有停止清理流程，不接受迟到的动作。

每轮打印 `[remote] roundtrip=...s actions=10x6`，包含图像上传、推理和返回。仍为同步顺序：观察→推理→执行10步→重新观察，不进行异步动作拼接。不能把30 Hz动作频率解释为30 Hz重新推理。

本机和远程推理入口不能同时占用机器人或相机。2026-09-22 已在 GPU 2 使用合成观测完成 v11/2000 的 WebSocket 推理检查，连续3次返回有限的 `10×6` 动作；预热约13.42秒，后两次服务端本机往返约0.138–0.143秒（不含本机到服务器的网络传输）。记录：`threading_real/pi05_openpi/outputs/v11_setup/remote_deployment_smoke.json`。测试服务已关闭，未启动相机或机器人，原训练持续运行。

历史 v4/H50 测量为平均0.157秒往返，仅作历史参考，不代表 v11/H10 的性能。v11 的实际耗时以客户端 `[remote] roundtrip=... actions=10x6` 日志为准。

## 定位同步问题：只增加诊断，不改变控制策略

在原本机部署命令上添加以下参数（仍由操作者手动启动机器人）：

```bash
  --sync-require-target \
  --max-cycles 3 \
  --sync-diagnostics /home/huiyuan/teleoperation/data/analysis/pi05_openpi_remote_eval/logs/sync_diagnostics.jsonl
```

不要同时修改动作幅度、刚度或超时，否则无法区分原因。`--sync-require-target` 沿用当前严格到位检查；诊断参数本身不改变控制策略。即使到位超时，日志也会保留等待期间的记录和最终 `wait_timeout`。

诊断记录包含相机读取起止、观测状态、推理起止、执行起点、计划目标、UDP最后成功发送序号与目标、发送线程状态，以及约20Hz的等待期间实测FK、关节速度和目标误差。所有时刻使用本机monotonic时钟；相机记录是读取时间，不是曝光时间；UDP sendto成功不代表控制端已收包。发送端manager的scheduled目标与真正已发送目标分别记录。

结束后离线汇总：

```bash
cd /home/huiyuan/teleoperation
threading_real/pi05_openpi/.venv-deploy/bin/python \
  threading_real/pi05_openpi/analyze_sync.py \
  data/analysis/pi05_openpi_remote_eval/logs/sync_diagnostics.jsonl
```

优先看以下字段：

- `observation_to_execution_net_motion_mm`：观测采样到执行起点之间的净移动；不是整个时段的路径长度，也不严格等于纯推理期间运动。
- `udp_sends_during_inference`：推理期间发送端是否仍在发送目标。
- `first_send_completed` 与 `last_wait`：发送完成时和最终等待时的误差、关节速度、最后已发送目标。
- `observed_hold_after_send_s`：发送结束后实际记录到的等待时长。
- `sender.send_error`、`sender_thread_alive`、`last_send_monotonic_s`：发送是否中断或停滞。

若发送持续、最终目标一致，而误差保持不降，下一步需在控制端增加收包序号/目标与 `O_T_EE` 记录，区分接收、末端定义和阻抗跟踪问题。当前诊断尚不能跨主机测量网络延迟或确认控制端末端坐标。
