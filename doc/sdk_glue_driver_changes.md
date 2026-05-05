# sdk_glue 底层驱动修改记录

本文档记录 HPM6E80 → HPM5361 移植过程中对 sdk_glue 底层驱动的修改。

---

## 1. DTS: hpm53xx.dtsi

**文件**: `d:\Zephyr_HPMicro\sdk_glue\dts\riscv\hpmicro\hpm53xx.dtsi`

### 1.1 时钟控制器节点

**改前**:
```dts
clk: clock-controller@f40c0000 {
    compatible = "hpmicro,hpm-pllv2";
    reg = <0xf40c0000 0x4000>;
    clock-frequency = <DT_FREQ_M(960)>;
    reg-names = "clk";
    #clock-cells = <3>;
    status = "okay";
};
```

**改后**:
```dts
clk: clock-controller@f40c0000 {
    compatible = "hpmicro,hpm-clock";
    reg = <0xf4000000 DT_SIZE_K(256)
           0xf40c0000 DT_SIZE_K(256)>;
    reg-names = "sysctl", "pllv2";
    #clock-cells = <3>;
    status = "okay";
};
```

**变更说明**:
- compatible 从 `hpm-pllv2` 改为 `hpm-clock`（与 HPM6E00/HPM6200 等使用 PLLCTL v2 的芯片一致）
- reg 从单一 PLL 寄存器区域改为 sysctl + pllv2 双区域，提供驱动所需的基地址
- reg-names 对应驱动 `DT_REG_ADDR_BY_NAME` 查找的 key

### 1.2 新增 PLL 节点（根级别）

新增 PLL0/PLL1 节点，兼容字符串 `hpmicro,hpm-pllv2-clock`:

```dts
pll0: pll0 {
    compatible = "hpmicro,hpm-pllv2-clock";
    pll-frequency = <720000000>;   /* VCO 频率 */
    pll-index = <0>;
    clk0 { src-name = <CLK_SRC_PLL0_CLK0>; div-i = <0>; div-p = <0>; };
    clk1 { src-name = <CLK_SRC_PLL0_CLK1>; div-i = <1>; div-p = <1>; };
    clk2 { src-name = <CLK_SRC_PLL0_CLK2>; div-i = <2>; div-p = <4>; };
};

pll1: pll1 {
    compatible = "hpmicro,hpm-pllv2-clock";
    pll-frequency = <800000000>;   /* VCO 频率 */
    pll-index = <1>;
    clk0 { src-name = <CLK_SRC_PLL1_CLK0>; div-i = <0>; div-p = <0>; };
    clk1 { src-name = <CLK_SRC_PLL1_CLK1>; div-i = <1>; div-p = <1>; };
    clk2 { src-name = <CLK_SRC_PLL1_CLK2>; div-i = <2>; div-p = <3>; };
    clk3 { src-name = <CLK_SRC_PLL1_CLK3>; div-i = <3>; div-p = <10>; };
};
```

PLL 频率和分频值对应 HPM SDK 预设:
| PLL | 输出 | 后分频 | 频率 |
|-----|------|--------|------|
| PLL0 clk0 | div-p=0 (÷1.0) | 720MHz |
| PLL0 clk1 | div-p=1 (÷1.2) | 600MHz |
| PLL0 clk2 | div-p=4 (÷1.8) | 400MHz |
| PLL1 clk0 | div-p=0 (÷1.0) | 800MHz |
| PLL1 clk1 | div-p=1 (÷1.2) | 667MHz |
| PLL1 clk2 | div-p=3 (÷1.6) | 500MHz |
| PLL1 clk3 | div-p=10 (÷3.0) | 267MHz |

### 1.3 新增晶振节点

```dts
osc24: osc24 {
    #clock-cells = <2>;
    compatible = "hpmicro,hpm-src-clock";
};
osc32: osc32 {
    #clock-cells = <2>;
    compatible = "hpmicro,hpm-src-clock";
};
```

### 1.4 hdma 兼容字符串修正

**改前**: `compatible = "hpmicro,hpm-dma";`
**改后**: `compatible = "hpmicro,hpm-dmav2";`

HPM5361 实际上使用 DMAv2（寄存器头文件为 `hpm_dmav2_regs.h`，类型为 `DMAV2_Type`），旧兼容字符串与硬件不匹配。

### 影响范围

- 仅 HPM53xx 系列 DTS，不影响其他芯片

---

## 2. PLL v2 时钟驱动

**文件**: `d:\Zephyr_HPMicro\sdk_glue\drivers\clock_control\clock_control_hpmicro_pllv2.c`

### 2.1 结构体成员名不一致

struct 定义:
```c
struct clock_pllv2_cfg {
    PLLCTLV2_Type *pll_base;    // 定义: pll_base
    SYSCTL_Type *sysctl_base;   // 定义: sysctl_base
    ...
};
```

代码中误用 `base`/`sysctl`（无 `_base` 后缀）:
```c
// 改前 (编译错误):
cfg->base                                 // → cfg->pll_base
cfg->sysctl                               // → cfg->sysctl_base

// 改后:
cfg->pll_base
cfg->sysctl_base
```

### 2.2 初始化宏字段名错误

```c
// 改前:
.base = (PLLCTL_Type *)DT_REG_ADDR_BY_NAME(CLOCK_CONTROLLER, pll),
.sysctl = (SYSCTL_Type *)DT_REG_ADDR_BY_NAME(CLOCK_CONTROLLER, sysctl),

// 改后:
.pll_base = (PLLCTLV2_Type *)DT_REG_ADDR_BY_NAME(CLOCK_CONTROLLER, pllv2),
.sysctl_base = (SYSCTL_Type *)DT_REG_ADDR_BY_NAME(CLOCK_CONTROLLER, sysctl),
```

### 2.3 具体修改点

| 行号 | 改前 | 改后 |
|------|------|------|
| 61 | `cfg->base` | `cfg->pll_base` |
| 70 | `cfg->sysctl` | `cfg->sysctl_base` |
| 71 | `cfg->base` | `cfg->pll_base` |
| 77 | `cfg->base` | `cfg->pll_base` |
| 108 | `.base = (PLLCTL_Type *)DT_REG_ADDR_BY_NAME(..., pll)` | `.pll_base = (PLLCTLV2_Type *)DT_REG_ADDR_BY_NAME(..., pllv2)` |
| 109 | `.sysctl =` | `.sysctl_base =` |

### 影响范围

- 所有使用 PLLCTL v2 的芯片（HPM6E00、HPM6200、HPM6800、HPM5300）
- 改前该驱动文件从未成功编译（struct 成员名错误 + CMake 大小写不匹配），改后正常编译
- 驱动逻辑未变，仅修正编译错误

---

## 3. CMakeLists.txt 大小写修正

**文件**: `d:\Zephyr_HPMicro\sdk_glue\drivers\clock_control\CMakeLists.txt`

**改前**:
```cmake
zephyr_library_sources_ifdef(CONFIG_CLOCK_CONTROL_HPMICRO_PLLv1 ...)
zephyr_library_sources_ifdef(CONFIG_CLOCK_CONTROL_HPMICRO_PLLv2 ...)
```

**改后**:
```cmake
zephyr_library_sources_ifdef(CONFIG_CLOCK_CONTROL_HPMICRO_PLLV1 ...)
zephyr_library_sources_ifdef(CONFIG_CLOCK_CONTROL_HPMICRO_PLLV2 ...)
```

Kconfig 生成的变量名为大写 `PLLV1`/`PLLV2`，CMake 条件中 `PLLv1`/`PLLv2`（小写 v）永不匹配。

### 影响范围

- 改前: 所有芯片的 PLLV1/PLLV2 驱动源文件都不参与编译（CMake 条件永不匹配）
- 改后: 有对应 Kconfig enabled 的芯片会编译相应源文件。PLLV1 影响 HPM67xx，PLLV2 影响 HPM6E00/HPM6200/HPM6800/HPM5300

---

## 4. Kconfig.soc: 补充 HAS_HPMSDK_PLLCTLV2

**文件**: `d:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\HPM5300\Kconfig.soc`

**改前**:
```kconfig
config SOC_SERIES_HPM5300
    bool
    select SOC_FAMILY_HPM
```

**改后**:
```kconfig
config SOC_SERIES_HPM5300
    bool
    select SOC_FAMILY_HPM
    select HAS_HPMSDK_PLLCTLV2
```

DTS 时钟节点更换 compatible 后，`CLOCK_CONTROL_HPMICRO_PLLV2` 不再自动 enabled，导致 PLLCTLV2 驱动源码不编译，HPM SDK 的 `hpm_clock_drv.c` 链接时找不到 PLLCTL v2 函数。在 SoC 级直接 select 确保 PLLCTLV2 驱动始终参与编译。

### 影响范围

- 仅 HPM5300 系列

---

## 5. flash_hpmicro.c: ARRAY_SIZE 重定义

**文件**: `d:\Zephyr_HPMicro\sdk_glue\drivers\flash\flash_hpmicro.c`

**改前**:
```c
#include "hpm_romapi.h"
#include <zephyr/kernel.h>
```

**改后**:
```c
#include "hpm_romapi.h"
#ifdef ARRAY_SIZE
#undef ARRAY_SIZE
#endif
#include <zephyr/kernel.h>
```

HPM SDK 的 `hpm_common.h` 定义了 `ARRAY_SIZE` 宏，Zephyr 的 `util.h` 也定义了同名的宏。HPM SDK 头先被包含，Zephyr 头后包含时触发重定义警告。`#undef` 后让 Zephyr 宏生效。

### 影响范围

- 所有使用此 flash 驱动的芯片（HPM6E00、HPM6200、HPM5300 等）
- 纯宏定义修复，不影响运行时行为
