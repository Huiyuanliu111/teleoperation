# PlanARP：required-only 与 full-then-truncate 比较方案

更新时间：2026-09-14。本文依据当前本地代码整理，区分已有功能与待实现方案。

**结论：可以用同一个 PlanARP checkpoint 做比较，无需重新训练策略。当前完整生成后截取的路径已存在，required-only、配对同步计时和两种模式的真机对照尚未完成。** 本文没有新的 PlanARP 性能实验结果。

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

这是最接近 ALOHA 比较流程的接口。已有 `--chunk-selector` 加载与执行前缀选择，`predict_action(..., visual_features=...)` 支持复用编码结果。但当前 MVT selector 路径显式只允许 `full_then_truncate`。

接入 required-only 后，配对测量只运行一次 selector、固定其选择，再分别解码。候选长度必须以实际 selector checkpoint 为准，不能直接套用 ALOHA 的 20/40/60。本文未确认某个 selector checkpoint 的真机有效性。


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

最小验证不需要重训策略，也不需要改变 plan_steps、horizon 或 action_chunk_size。策略代码支持长度控制后，应继续严格加载同一个 checkpoint。

建议实现顺序：固定 h 的按需生成与前缀测试 → 离线同步计时排错 → 选定一种调度器接入 → 实际调用配对计时 → 独立真机闭环对照。

## 6. 安全检查与比较公平性

当前部署端先检查生成的整个候选动作序列，超出原始平移/旋转阈值时中止，再积分、裁剪并选择执行前缀。

full 有 10 步可检查，required 精细阶段只有 4 步。full 可能因为第 8 步异常中止，required 则没有生成第 8 步。因此，即使前三步一致，两者的中止行为也可能不同。

实现前需要明确实验协议：

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

## 8. 当前实现状态

| 项目 | 状态 | 证据或限制 |
|---|---|---|
| 4 个计划位姿 + 10 步动作生成 | 已实现 | 策略推理循环仍按 self.horizon 完整生成 |
| 执行完整生成序列的前缀 | 已实现 | 固定步数、endpoint schedule、MVT selector 均有路径 |
| endpoint 10→3 调度 | 已实现 | 依赖当前位置及完整预测路径，精细状态保持 |
| MVT selector 先选 h | 接口已实现 | 可共享视觉特征；当前只允许 full_then_truncate |
| visual_features 缓存参数 | 已实现 | 可供后续配对解码复用 |
| PlanARP required-only | **未实现** | 当前没有 requested_steps 控制实际生成长度 |
| 按组取整并验证前缀一致 | **未实现** | 需要补充策略和 checkpoint 测试 |
| 两种模式的安全检查协议 | **待确定** | 整段候选检查与生成长度不同存在冲突 |
| PlanARP 配对 CUDA 同步计时 | **未实现/未测量** | 现有 MVT selector 路径使用 CPU monotonic 计时，不等于文档中的逐调用同步计时 |
| PlanARP 两模式真机成功率对照 | **未完成** | 不能从 ALOHA 结果或监督验证 loss 推断 |

部署脚本已有 `--prediction-mode` 参数，但这不代表 MVT PlanARP 已支持 required-only。带 MVT selector 时非 full 模式目前会显式报错；不带 selector 时，也不能仅凭传入该参数就宣称策略进行了按需生成。

## 9. 相关代码与文件

- [MVT PlanARP 策略](../threading_real/threading_task/mvt_arp_policy.py)
- [ARP 分组生成实现](../threading_real/pushbox/arp.py)
- [Cartesian 真机入口](../threading_real/scripts/deployment/cartesian.py)
- [MVT selector 与共享视觉特征](../threading_real/chunk_selector/mvt_features.py)
- [Endpoint schedule](../threading_real/scripts/deployment/endpoint_schedule.py)
- [Endpoint 调度 YAML](../threading_real/calibration/threading_combined_80_execution.yaml)
- [PlanARP v2 训练 YAML](../threading_real/pushbox/configs/threading_combined_80_mvt_planarp_v2.yaml)
- [ALOHA 比较参考](/home/huiyuan/arp/aloha/doc/comparison.md)

本文仅记录方案与状态，未修改推理行为、重启训练或启动真机实验。
