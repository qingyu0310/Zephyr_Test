# CAN 波特率 Bug 分析

## 现象

DTS overlay 中设置 `bitrate = <1000000>`（1 Mbps），但实际 CAN 运行在 125 kbps。

```
CAN: clock=80000000 baud=125000 NBTP=0x9e01ee4f
```

NBTP = 0x9e01ee4f 解码：

| 域 | 位域 | 原始值 | 实际值 |
|---|---|---|---|
| NBRP (预分频) | [24:16] | 1 | 2 |
| NTSEG1 | [15:8] | 0xEE = 238 | 239 |
| NTSEG2 | [6:0] | 0x4F = 79 | 80 |

```
baud = 80MHz / (2 × (1 + 239 + 80)) = 80MHz / 640 = 125000 ✓ (符合 NBTP)
```

## 根因

**文件:** `d:\Zephyr_HPMicro\sdk_glue\drivers\can\mcan_hpmicro.c` 第 1038 行

```c
#define HPM_MCAN_INIT(n)                                                \
    PINCTRL_DT_INST_DEFINE(n);                                          \
    static void hpm_mcan_irq_config_func##n(const struct device *dev); \
                                                                        \
    static const struct hpm_mcan_config hpm_mcan_cfg_##n = {            \
        .common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, 1000000),  /* ← BUG: 应该是 n */ \
        .base = (MCAN_Type*)DT_INST_REG_ADDR(n),                       \
        ...
```

宏 `HPM_MCAN_INIT` 的参数是 `n`（DT 实例编号），但 `CAN_DT_DRIVER_CONFIG_INST_GET` 的参数是 **`inst`** 而不是 `n`。

`inst` 不是该宏的参数，也不是任何地方 `#define` 的宏，所以预处理后它作为**未定义标识符**被传递到 `CAN_DT_DRIVER_CONFIG_INST_GET`，最终展开成：

```c
DT_PROP_OR(undefined_node, bitrate, CONFIG_CAN_DEFAULT_BITRATE)
```

因为节点 ID 未定义，`DT_NODE_HAS_PROP` 返回 false（`IS_ENABLED(undefined_macro)` = 0），于是 `DT_PROP_OR` 直接使用了**第三个参数（默认值）**——而 Zephyr 通过 `COND_CODE_1` 取 else 分支，最终拿到的是 `CONFIG_CAN_DEFAULT_BITRATE`。

**DTS 中的 `bitrate = <1000000>` 从未被读取。**

## 为何编译不报错

`DT_PROP_OR` 的实现：

```c
#define DT_PROP_OR(node_id, prop, default_value) \
    COND_CODE_1(DT_NODE_HAS_PROP(node_id, prop),  \
                (DT_PROP(node_id, prop)),          \
                (default_value))
```

`DT_NODE_HAS_PROP` 底层用 `IS_ENABLED(macro)` 检查属性是否存在。当 node_id 未定义时 `IS_ENABLED` 返回 0，`COND_CODE_1(0, ...)` 走 else 分支返回默认值。整个过程不触发编译错误。

## 修复

把 `inst` 改为 `n`：

```diff
- .common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, 1000000),
+ .common = CAN_DT_DRIVER_CONFIG_INST_GET(n, 0, 1000000),
```

**文件:** `d:\Zephyr_HPMicro\sdk_glue\drivers\can\mcan_hpmicro.c`

## 验证

修复后 `CAN_DT_DRIVER_CONFIG_INST_GET(0, 0, 1000000)` 正确展开为 `DT_N_INST_0_hpmicro_hpm_mcan`，从 DTS 读到 `bitrate = 1000000`，NBTP 重新计算：

```
80MHz → prescaler=1, TSEG1=59, TSEG2=20, total Tq=80
baud = 80MHz / (1 × (1 + 59 + 20)) = 80MHz / 80 = 1000000 = 1 Mbps ✓
```

## 相关配置

```
# e:\Zephyr_project\HPMicro\blinky\build\zephir\.config
CONFIG_CAN_DEFAULT_BITRATE=125000

# e:\Zephyr_project\HPMicro\blinky\projects\boards\hpm6e00evk.overlay（本应生效但未被读到）
bitrate = <1000000>;
```
