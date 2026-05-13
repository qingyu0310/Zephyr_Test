# Zephyr 4.4.0 构建问题记录

## 背景

本仓库在构建 STM32 目标（`stm32f4_disco`）时遇到了 Zephyr 4.4.0 的两个构建系统层面的问题。本项目为 out-of-tree 应用（无 west workspace，通过 `ZEPHYR_MODULES` 传入外部模块）。

---

## 问题 1：CMSIS 模块 Kconfig 冲突

### 现象

```text
warning: HAS_CMSIS_CORE (defined at modules\cmsis\Kconfig:7, modules\cmsis_6\Kconfig:8)
  has direct dependencies 0 || 0 with value n, but is currently being y-selected
```

### 原因

Zephyr 4.4 新增了 `modules/cmsis_6/` 模块（CMSIS v6 支持），但保留了旧的 `modules/cmsis/`。两个模块的 Kconfig 都定义了 `HAS_CMSIS_CORE`，且通过 `modules/Kconfig` 第 136 行的 `if 0` 回退块被加载：

```kconfig
if 0
osource "modules/*/Kconfig"
endif
```

kconfiglib 对 `osource` 的实现会在 `if 0` 内部仍然进入文件并解析符号，只是设依赖为 `0`。于是 `HAS_CMSIS_CORE` 依赖为 `n`，但 `HAS_STM32CUBE`（`modules/Kconfig.stm32`）和 `CPU_CORTEX_M`（`arch/arm/core/Kconfig`）对其执行 `select y`，Kconfig 认为这是错误。

### 解决方案（项目 Kconfig 中预定义符号）

```kconfig
config HAS_CMSIS_CORE
    bool

source "Kconfig.zephyr"
```

将组合依赖从 `0 || 0` 变为 `y || 0 || 0 = y`，消除冲突。

---

## 问题 2：模块 CMakeLists.txt 与 Kconfig 的加载时序问题

### 现象

- `stm32f4xx.h` 找不到（ST HAL 头文件缺失）
- `cmsis_core.h` 找不到（CMSIS 头文件缺失）
- `SystemCoreClock`、`HAL_RCC_GetSysClockFreq` 等链接错误（STM32Cube 源文件缺失）
- `stm32_ll_pwr.h`、`stm32_ll_gpio.h` 等 LL 头文件缺失

### 原因

Zephyr 4.4 的 cmake 模块加载顺序为：

```
1. zephyr_module  → 加载外部模块的 CMakeLists.txt ──┐
2. boards         → 板级配置                          │ CONFIG_* 不可用
3. shields                                             │
4. snippets                                            │
5. hwm_v2                                              │
6. configuration_files                                  │
7. dts                                                 │
8. kconfig         → 处理 Kconfig，设置 CONFIG_xxx 变量 ┘
9. arch
10. soc
```

外部模块的 CMakeLists.txt 在步骤 1 中通过 `add_subdirectory` 加载，此时 Kconfig 尚未处理（步骤 8），所有 `CONFIG_*` 变量均为空。

受影响的关键模块代码：

- **STM32 HAL 模块** `CMakeLists.txt`：
  ```cmake
  add_subdirectory_ifdef(CONFIG_HAS_STM32CUBE stm32cube)  # 永不为真
  add_subdirectory_ifdef(CONFIG_HAS_STM32LIB lib)
  ```
  导致 `stm32cube/` 子目录不被加载，HAL/LL 源文件和头文件均不参与编译。

- **CMSIS_6 模块** `CMakeLists.txt`：
  ```cmake
  if(CONFIG_CPU_CORTEX_M)
    add_subdirectory(...)
    zephyr_include_directories(.)          # 不执行
  endif()
  ```
  导致 `cmsis_core.h` 不在包含路径中。

### 为什么标准 west workspace 不受影响

在正确初始化的 west workspace 中，Kconfig 值的预处理/缓存机制弥补了这个时序间隙。本项目为 out-of-tree 应用，不走 workspace，因此暴露了该问题。

### 解决方案（不完美）

在 `find_package(Zephyr)` 前预置 CONFIG_ CMake 缓存变量：

```cmake
set(CONFIG_HAS_STM32CUBE y CACHE BOOL "" FORCE)
set(CONFIG_CPU_CORTEX_M y CACHE BOOL "" FORCE)
```

但需注意用 `y` 而非 `1`（Kconfig 布尔值要求），且这些值会被后续 Kconfig 覆盖。

同时需手动添加头文件路径和编译定义，作为模块 CMakeLists.txt 不执行时的回退：

```cmake
zephyr_include_directories("D:/Zephyr/zephyr/modules/cmsis_6")
zephyr_include_directories("D:/Zephyr/modules/hal/cmsis_6/CMSIS/Core/Include")
zephyr_include_directories("D:/Zephyr/modules/hal/stm32/stm32cube/stm32f4xx/soc")
zephyr_include_directories("D:/Zephyr/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include")
zephyr_include_directories("D:/Zephyr/modules/hal/stm32/stm32cube/common_ll/include")
zephyr_compile_definitions(-DSTM32F407xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER)
```

> 这些路径和定义是 SoC 相关的，多目标构建时需要用 `if(CONFIG_SOC_FAMILY_STM32)` 包裹。

---

## 总结

| 问题 | 严重程度 | 修复难度 | 是否 Zephyr Bug |
|------|----------|----------|-----------------|
| CMSIS Kconfig 冲突 | 阻断编译 | 低 | 是（4.4 新增 cmsis_6 时遗留） |
| 模块加载时序 | 阻断编译 | 中 | 是（设计缺陷，out-of-tree 场景暴露） |

两个问题在 Zephyr 4.3 中均不存在：4.3 没有 `cmsis_6` 模块，且模块加载时序处理正常。

降级到 4.3 是当前最干净的解决方案，无需任何 CMakeLists.txt workaround 代码。
