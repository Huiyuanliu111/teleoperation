# PlanARP：required-only 与 full-then-truncate 实现与比较记录

更新时间：2026-09-14。本文记录本地实现、验证结果与尚未完成的实验。

**结论：可以用同一个 PlanARP checkpoint 做比较，无需重新训练策略。两种推理模式、固定长度 / MVT selector / endpoint 调度接入和离线配对计时脚本均已实现；默认仍为 full_then_truncate。真机对照尚未完成。** 本文没有新的 PlanARP 性能实验结果。

## 1. 比较目标与参考

参考 [ALOHA 比较文档](/home/huiyuan/arp/aloha/doc/comparison.md)：先确定执行长度 h，再在相同观测与相同 h 上比较完整生成和按需生成，分别测量推理耗时与闭环任务效果。

ALOHA 文档报告了逐调用配对同步计时约 16.08% 的平均加速，以及该样本量下未检出显著成功率差异。这些结果仅属于那组 ALOHA 实验，不能当作 PlanARP 的预期加速或效果保证；未检出显著差异也不等于证明两种策略等价。

PlanARP 的比较目标是：保持权重、观测、计划、执行长度及控制参数一致，检查省略不执行的后续动作组，能否减少推理耗时，同时保持执行前缀和实际插入表现。

## 2. 当前策略结构

基线使用 [PlanARP v2 配置](../threading_real/pushbox/configs/threading_combined_80_mvt_planarp_v2.yaml)：

| 参数 | 当前值 |
|---|---|
| 数据 | `threading_combined_80_mvt_7p5hz.h5`，7.5 Hz |
| 动作窗口 horizon | 10 步 |
| 计划位姿 plan_steps | 4 个 |
| 计划顺序 | `reverse_plan=true`，远到近 |
| 密集动作分组 | `action_chunk_size=2` |
| 夹爪预测 | 关闭，输出夹爪增量为 0 |
| 推理采样 | 当前为 `sample=False` |

生成顺序为：

```text
当前点云与 TCP 控制点
    → [G1, G2, G3, G4]
    → [A1, A2] → [A3, A4] → [A5, A6] → [A7, A8] → [A9, A10]
```

每个位姿由三个控制点在两个视图中的投影表示，即 6 个空间 token。计划共 24 个 token，完整动作共 60 个 token；另有 6 个当前状态 prompt token。

计划标签由同一段 10 步目标控制点轨迹重采样得到，对应第 1、4、7、10 个目标位姿。它是同窗口的粗到细预测，不是覆盖更远未来的长期计划。本次比较不改变这个定义。

可使用的已训练 checkpoint：

```text
threading_real/outputs/threading_combined_80_mvt_planarp_v2/20260912_121546/checkpoints/epoch=0004-val_loss=10.198.ckpt
```

该文件是第 5 轮结束时保存的 checkpoint。验证 loss 属于真实计划条件下的监督损失，不等同于生成式推理位姿误差或真机成功率。

## 3. 两种模式的严格定义

### full_then_truncate

1. 编码当前观测。
2. 生成全部 4 个计划位姿。
3. 生成全部 10 步密集动作。
4. 执行已选定的前 h 步。

### required_only：保留完整动作组

1. 使用相同观测和执行长度 h。
2. 生成相同的全部 4 个计划位姿。
3. 只生成覆盖 h 所必需的完整动作组。
4. 执行前 h 步。

设动作组大小为 c，生成步数为：

```text
generated_steps = min(horizon, ceil(h / c) * c)
```

当前 c=2，所以执行 3 步时生成 4 步。原因是 A3 和 A4 在同一组内可以交换隐藏表示；直接删除 A4 会改变 A3 的注意力上下文。保留完整组能避免这一结构变化。

这与 ALOHA 的 `h+1` 不同：ALOHA 的额外 position 来自它的动作对齐方式，PlanARP 不需要固定加一。若以后研究严格生成 h 步，应单独标为另一种实验，不与完整组版本混为一谈。

| 执行 h 步 | full 生成动作步数 | required 生成动作步数 | full / required 生成 token 数 |
|---:|---:|---:|---:|
| 2 | 10 | 2 | 84 / 36 |
| 3 | 10 | 4 | 84 / 48 |
| 4 | 10 | 4 | 84 / 48 |
| 6 | 10 | 6 | 84 / 60 |
| 8 | 10 | 8 | 84 / 72 |
| 10 | 10 | 10 | 84 / 84 |

token 数包含计划、不含共同的 6 个 prompt token。执行 3 步时生成 token 减少 42.9%，ARP 生成组数由 6 组降为 3 组，但渲染、视觉编码与计划生成成本仍在，不能将 token 减少比例等同于延迟改善比例。

在相同观测、确定性推理、保留完整组且实现正确的条件下，已有前缀的模型依赖关系应保持一致。是否达到数值一致，需要用 checkpoint 实测，不能只凭结构断言逐位相等。

## 4. 如何在生成前决定 h

### 现有 MVT selector

当前 [mvt_features.py](../threading_real/chunk_selector/mvt_features.py) 已实现：

```text
当前观测 → 一次 MVT 编码 → selector 选择 h
                         → 使用同一视觉特征生成完整动作
```

这是最接近 ALOHA 比较流程的接口。已有 `--chunk-selector` 加载与执行前缀选择，`predict_action(..., visual_features=...)` 支持复用编码结果。现在支持两种模式，将 selector 选出的 h 传给策略。在批量接口中取 batch 内最大的 h，真机使用 batch size 1。

后续 live 配对测量应只运行一次 selector、固定其选择，再分别解码。当前新增脚本使用离线观测和指定 h，不运行 selector。候选长度必须以实际 selector checkpoint 为准，不能直接套用 ALOHA 的 20/40/60。本文未确认某个 selector checkpoint 的真机有效性。


## 5. 实现难度与工作项

难度是基于当前代码的工程判断，不是已经测量的开发耗时。

| 工作项 | 难度 | 主要内容 |
|---|---|---|
| 策略按需生成 | 中 | 增加模式和 requested_steps；保持完整计划；按动作组取整；调整 token、特征上下文和位姿解码循环的长度 |
| 输出接口与元数据 | 低至中 | 区分 horizon、requested、generated、executed；避免下游继续假设 action_pred 恒为 10 步 |
| 前缀一致性测试 | 中 | 覆盖奇偶执行长度、两步同组、计划输出、姿态差分、夹爪为零，以及 h=10 无差别基线 |
| MVT selector 接入 | 中 | 将先选出的 h 传给解码器；共享视觉编码；移除当前仅支持 full 的限制前补齐测试 |
| endpoint schedule 接入 | 中 | 推理前识别已锁定的精细模式；粗阶段仍完整生成；精细阶段不能再调用要求 10 个目标的 select 接口 |
| 安全检查口径 | 中，必须明确 | 当前整段候选动作检查与不同生成长度存在冲突，见下节 |
| 逐调用同步计时 | 中 | CUDA event、同步、交替顺序、视觉缓存口径、逐调用日志与前缀差异 |
| 真机闭环对照 | 高 | 初始状态复现、接触条件差异、独立运行两模式、成功判定、足够样本与统计分析 |

本次未重训策略，未改变 plan_steps、horizon 或 action_chunk_size。策略支持长度控制，同一个 checkpoint 的参数结构保持兼容。

固定 h、两种调度器接入和离线配对工具已完成；实际闭环调用配对计时与独立真机对照仍待开展。上表难度是工程判断，完成状态见第 8 节。

## 6. 安全检查与比较公平性

当前部署端先检查生成的整个候选动作序列，超出原始平移/旋转阈值时中止，再积分、裁剪并选择执行前缀。

full 有 10 步可检查，required 精细阶段只有 4 步。full 可能因为第 8 步异常中止，required 则没有生成第 8 步。因此，即使前三步一致，两者的中止行为也可能不同。

本次采用“检查全部已生成动作”的协议，保留原阈值。固定步数的 MVT 路径也统一从 action_pred 获取全部生成动作，full 模式可能比此前只检查截取后的 action 更早中止。成功周期记录生成、检查、选择与下发步数；原始动作阈值中止会记录异常步和 executed_steps=0。未生成的后缀不在 required 的检查范围内，闭环比较必须报告这一差异。

后续实验需注意：

- 延迟与输出前缀比较可以先独立完成，不要求实际执行动作。
- 若保留“检查全部生成动作”，必须将检查长度、异常所在步和中止次数分别记录，将差异作为混杂因素报告。
- 若设计统一执行前缀的安全检查方案，必须明确记录这是部署语义变更，并单独验证；不能为了获得加速而悄悄取消已有检查或提高阈值。
- 不能声称 required-only 检查了没有生成的后续动作。若要求两种模式都检查完整 10 步，就必须生成完整轨迹，该部分调用也就没有按需生成收益。

## 7. 计时与效果评估

### 逐调用配对计时

使用同一当前观测、同一 checkpoint、同一 h 和调度状态，交替执行 full/required 的先后顺序。CUDA 区间使用 event，并在读取时间前同步。只执行选定主模式的动作推进环境，另一模式仅作配对测量。

至少分别报告：

1. 完整策略调用：MVT 渲染、视觉编码、计划生成、动作生成和动作解码。
2. 共享视觉特征后的计划与动作生成耗时。
3. 如进一步复用计划，单独报告动作解码耗时，不能与包含计划的时间混用。
4. selector 耗时、CPU 预处理耗时和真实 episode wall time，明确各自边界。

两种模式使用对等的缓存条件。不能一个模式复用视觉或计划、另一个重新计算。共享特征的微观计时与包含编码的整次调用计时应分开采集和命名。

建议逐调用记录：episode、cycle、观测时间戳、模式、调度状态、requested_steps、generated_steps、executed_steps、动作组数、plan/action token 数、运行顺序、各段耗时、计划差异、前缀位置/旋转差异和安全检查结果。

比较旋转时使用 SO(3) 测地角误差，不仅比较旋转向量各分量。热图 argmax 的微小数值差异可能变成离散像素变化，需同时检查原始控制点与最终 Cartesian delta。

### 真机闭环效果

配对解码计时运行只能给出主执行模式的成功率，不能同时得到两个模式的闭环成功率。

两种模式需分别实际运行，并尽量匹配初始 TCP、盒子抓取、孔位和场景条件，交替测试顺序。真机没有仿真式严格 paired seeds，应记录复位误差和实际配对方式。

报告成功率及区间、完成时间、policy 调用次数、实际执行步数、精细阶段耗时、安全中止次数。只有预先定义了有效配对的 trials，才使用配对成功率检验；不能把不相关的真机 episodes 强行配对。

## 8. 当前实现状态与使用方式

| 项目 | 状态 | 证据或限制 |
|---|---|---|
| full_then_truncate | 已实现，默认 | 完整计划 + horizon 动作，执行所选前缀 |
| required_only | 已实现 | 完整计划 + 向上取整到完整动作组 |
| 固定执行长度 | 已接入 | --execute-steps 传入 requested_steps |
| MVT selector | 两模式已接入 | 先选 h，两者共享一次视觉编码 |
| endpoint 10→3 | 两模式已接入 | 粗阶段和首次进入细阶段的调用生成 10；锁定 fine 后 required 生成 4、选择 3 |
| 前缀一致性 | 单元测试通过 | 真实 ARP decoder，覆盖奇偶 h、不同分组、计划和夹爪配置 |
| 安全检查 | 已统一口径 | 检查全部已生成动作；两模式检查长度不同 |
| 离线配对计时 | 工具已实现 | CUDA event + 同步 / CPU wall time；两种缓存口径分开 |
| live selector 配对计时 | 未实现 | 离线脚本不能替代真实闭环调用分布 |
| 真机成功率对照 | 未开展 | 尚无 PlanARP GPU 加速或真机效果结论 |

策略接口：

```python
prediction = policy.predict_action(
    obs,
    prediction_mode="required_only",  # 或 full_then_truncate
    requested_steps=3,
    # visual_features=visual,  # 可选，两模式需采用相同缓存条件
)
```

`requested_steps` 为 1 到 horizon 的整数，省略时采用策略的 n_action_steps。
`action_pred`、`target_control_points` 的长度为 generated_steps；`action` 为 requested_steps。
`plan_control_points` 保持完整。`prediction_diagnostics` 返回模式、requested_steps、generated_steps、计划 / 动作 token 数和动作组数。

在现有 Cartesian 部署命令中切换以下参数即可，checkpoint 与其他控制参数保持相同：

```bash
# 固定执行 3 步，完整生成 10 步
--prediction-mode full_then_truncate --execute-steps 3 --trace-output /tmp/planarp_full.jsonl

# 固定执行 3 步，按需生成 4 步
--prediction-mode required_only --execute-steps 3 --trace-output /tmp/planarp_required.jsonl
```

使用已有 `--chunk-selector` 时，由 selector 决定 h。使用 endpoint schedule 时，由调度状态决定 h：首次跨入精细区域仍需完整预测路径，该调用 requested/generated=10、selected=3；后续 required 调用 requested=3、generated=4、selected=3。reset 清除 fine 锁定。两种调度器不能同时使用。

trace 的 executed_steps 表示本周期计划下发的前缀长度（dry-run 或原始动作拒绝为 0），不表示机器人反馈确认完成的步数。轨迹中断或异步覆盖时不能将它当作物理完成步数。两种模式仍对所有已生成候选执行原有原始阈值检查、积分和裁剪。

### 离线复现命令

从 threading_real 目录运行；不连接机器人：

```bash
OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 /home/huiyuan/miniconda3/envs/arp/bin/python \
  scripts/diagnostics/compare_planarp_prediction_modes.py \
  --checkpoint outputs/threading_combined_80_mvt_planarp_v2/20260912_121546/checkpoints/epoch=0004-val_loss=10.198.ckpt \
  --dataset /home/huiyuan/teleoperation/data/datasets/threading_combined_80_mvt_cam1_7p5hz.h5 \
  --device cpu --samples 2 --requested-steps 3 10 --warmup 1 \
  --output outputs/planarp_required_only_check/paired_cpu.json
```

原 checkpoint 记录的训练数据路径已不存在，此处显式使用当前 cam1 文件；这是输出一致性抽查，不是原验证集效果复现。GPU 空闲时可改成 `--device cuda:0` 并增加 samples，使用不同输出文件。脚本交替解码顺序，分别记录包含视觉编码的整次调用、共享视觉后的计划与动作生成；不复用计划。JSON 保存逐调用生成量、平移差异、SO(3) 旋转差异、控制点、计划与夹爪差异。

### 2026-09-14 验证记录

- 上述 checkpoint 以 model 权重加载，220/220 参数键匹配。
- 使用 cam1 数据集划分后的两个观测：episode_000000 / frame 0、episode_000061 / frame 118；h=3 和 h=10，各测两种计时口径，共 8 组配对。
- CPU 上全部配对的执行前缀平移、SO(3) 旋转、控制点、夹爪以及完整计划差异均为 0。h=3 时 full / required 分别生成 10 / 4 步；h=10 时都生成 10 步。
- 这是两个观测的正确性抽查，不能证明所有观测、CUDA 数值路径或闭环轨迹都一致。当前 GPU 有其他任务，本次没有进行 GPU 性能测量；JSON 中 CPU 时间仅供排错。
- 相关测试共 **63 passed**，包含策略、模式路由、endpoint schedule、Cartesian 部署和 selector；git diff --check 通过。
- [8 组原始配对结果](../threading_real/outputs/planarp_required_only_check/paired_cpu.json)。该 outputs 文件是本地生成结果，未保证纳入版本控制。

验证命令（threading_real 目录）：

```bash
OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=. /home/huiyuan/miniconda3/envs/arp/bin/python -m pytest \
  tests/test_mvt_arp.py tests/test_mvt_prediction_modes.py \
  tests/test_endpoint_schedule.py tests/test_deploy_threading_real_cartesian.py \
  tests/test_chunk_selector.py -q
```

## 9. 相关代码与文件

- [MVT PlanARP 策略](../threading_real/threading_task/mvt_arp_policy.py)
- [ARP 分组生成实现](../threading_real/pushbox/arp.py)
- [Cartesian 真机入口](../threading_real/scripts/deployment/cartesian.py)
- [MVT selector 与共享视觉特征](../threading_real/chunk_selector/mvt_features.py)
- [Endpoint schedule](../threading_real/scripts/deployment/endpoint_schedule.py)
- [Endpoint 调度 YAML](../threading_real/calibration/threading_combined_80_execution.yaml)
- [PlanARP v2 训练 YAML](../threading_real/pushbox/configs/threading_combined_80_mvt_planarp_v2.yaml)
- [ALOHA 比较参考](/home/huiyuan/arp/aloha/doc/comparison.md)

本次实现了推理模式切换与离线比较工具，未重启训练或启动真机实验。

- [离线配对脚本](../threading_real/scripts/diagnostics/compare_planarp_prediction_modes.py)
- [部署模式测试](../threading_real/tests/test_mvt_prediction_modes.py)
