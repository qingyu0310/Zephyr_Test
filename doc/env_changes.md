# 环境修改记录

## 概述

本项目为 out-of-tree Zephyr 应用（无 west workspace），原目标平台为 HPMicro 系列 MCU。在增加 STM32F4 支持时遇到了 Zephyr 版本兼容问题，最终将 Zephyr 从 v4.4.0 降级到 v4.3.0，Zephyr SDK 从 1.0.1 降级到 0.16.8，并做了若干修补。

---

## 1. Zephyr 版本变更

### 1.1 主仓库

| 项目 | 旧值 | 新值 |
|------|------|------|
| Git tag | `v4.4.0` | `v4.3.0` |
| Zephyr base | `D:\Zephyr\zephyr` | 不变 |
| 切换方式 | — | `git checkout v4.3.0` |

操作：

```powershell
cd D:\Zephyr\zephyr
git fetch origin --tags          # 拉取 v4.3.0 tag（之前只有 v4.4.0）
git checkout v4.3.0
```

### 1.2 外部模块同步

```powershell
cd D:\Zephyr
west update
```

`west update` 根据 `D:\Zephyr\zephyr\west.yml`（v4.3.0 版本）同步所有外部模块：

| 模块 | 路径 |
|------|------|
| hal_stm32 | `D:\Zephyr\modules\hal\stm32` |
| CMSIS (旧) | `D:\Zephyr\modules\hal\cmsis` |
| CMSIS_6 | `D:\Zephyr\modules\hal\cmsis_6`（v4.3 manifest 也存在此模块） |

### 1.3 降级原因

v4.4.0 存在两个问题：

**问题 A — CMSIS 模块 Kconfig 冲突**

v4.4 新增 `modules/cmsis_6/` 但仍保留 `modules/cmsis/`，两个 Kconfig 都定义 `HAS_CMSIS_CORE`，且通过 `modules/Kconfig` 第 136 行的 `if 0` 回退块加载，导致符号依赖为 `n` 却被其他模块 `select y`，Kconfig 阻断编译。

**问题 B — 模块 CMakeLists.txt 与 Kconfig 加载时序**

Zephyr 的 cmake 模块加载顺序：

```
1. zephyr_module  → 加载外部模块的 CMakeLists.txt  ← CONFIG_* 不可用
2. boards
3. shields/snippets/hwm_v2/configuration_files/dts
8. kconfig         → 处理 Kconfig，设置 CONFIG_xxx  ← 此时才可用
```

外部模块（hal_stm32、CMSIS_6）的 CMakeLists.txt 使用 `add_subdirectory_ifdef(CONFIG_XXX dir)` 和 `if(CONFIG_XXX)` 做条件判断，但此时 Kconfig 尚未处理，所有 `CONFIG_*` 变量为空，导致：

- `hal_stm32/CMakeLists.txt` 中 `add_subdirectory_ifdef(CONFIG_HAS_STM32CUBE stm32cube)` → 永假 → STM32Cube HAL 源文件不参与编译
- `hal_cmsis_6/CMSIS/CMakeLists.txt` 中 `add_subdirectory_ifdef(CONFIG_HAS_CMSIS_CORE_M Core)` → 永假 → CMSIS Core 头文件路径不加入

此问题在标准 west workspace 中被 pre-cache 机制掩盖，out-of-tree 项目暴露。

v4.3.0 同样存在此时序问题（影响较小），但可通过下面第 4 节的方案绕过。

---

## 2. Zephyr SDK 变更

| 项目 | 旧值 | 新值 |
|------|------|------|
| SDK 版本 | 1.0.1 | **0.16.8** |
| SDK 路径 | `D:\Zephyr\zephyr-sdk-1.0.1` | `D:\Zephyr\zephyr-sdk-0.16.8` |

### 2.1 下载方式

从 https://github.com/zephyrproject-rtos/sdk-ng/releases/tag/v0.16.8 下载 `zephyr-sdk-0.16.8_windows-x86_64.7z`，解压到 `D:\Zephyr\zephyr-sdk-0.16.8`。

### 2.2 降级原因

SDK 1.0.1 的 `Zephyr-sdkConfigVersion.cmake` 中声明了最低兼容版本为 1.0：

```cmake
set(ZEPHYR_SDK_MINIMUM_COMPATIBLE_VERSION 1.0)
if(PACKAGE_FIND_VERSION VERSION_LESS ZEPHYR_SDK_MINIMUM_COMPATIBLE_VERSION)
    set(PACKAGE_VERSION_COMPATIBLE FALSE)
```

Zephyr v4.3 请求 SDK 0.16.x（< 1.0），因此 SDK 1.0.1 自报不兼容，CMake 拒绝使用。SDK 0.16.8 与 v4.3 匹配。

### 2.3 CMakeLists.txt 对应修改

```cmake
set(ZEPHYR_SDK_INSTALL_DIR "D:/Zephyr/zephyr-sdk-0.16.8" CACHE PATH "")
```

### 2.4 工具链变更

SDK 0.16.8 使用的编译器版本为 GCC 12.2.0（SDK 1.0.1 为 GCC 14.3.0）。

---

## 3. CMakeLists.txt 修改

文件：`E:\Zephyr_project\HPMicro\blinky\CMakeLists.txt`

### 完整当前内容

```cmake
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(ACTIVE_PRJ "test" CACHE STRING "Active project")
set(PROJ_DIR projects)
set(CONFIG_SYM PRJ_TEST)

# sdk_glue 路径（HPM 板级/SOC/DTS 定义）
if(DEFINED ENV{SDK_GLUE_DIR})
  set(SDK_GLUE_DIR "$ENV{SDK_GLUE_DIR}")
else()
  set(SDK_GLUE_DIR "D:/Zephyr_HPMicro/sdk_glue")
endif()
list(APPEND BOARD_ROOT "${SDK_GLUE_DIR}")
list(APPEND SOC_ROOT "${SDK_GLUE_DIR}")
list(APPEND DTS_ROOT "${SDK_GLUE_DIR}")
list(APPEND DTS_ROOT "${SDK_GLUE_DIR}/dts")
list(APPEND DTS_ROOT "/d/Zephyr/modules/hal/stm32")
set(DTS_EXTRA_CPPFLAGS "-ID:/Zephyr/dts/common;-ID:/Zephyr/modules/hal/stm32/dts")

file(GLOB OVERLAY_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/boards/*/${BOARD}/${BOARD}.overlay)
if(OVERLAY_FILES)
  list(POP_FRONT OVERLAY_FILES DTC_OVERLAY_FILE)
endif()

file(GLOB PRJ_CONF_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/boards/*/${BOARD}/${BOARD}.conf)
if(PRJ_CONF_FILES)
  list(POP_FRONT PRJ_CONF_FILES EXTRA_CONF_FILE)
endif()

set(ZEPHYR_SDK_GLUE_MODULE_DIR "${SDK_GLUE_DIR}" CACHE PATH "")
set(ZEPHYR_MODULES "${SDK_GLUE_DIR};D:/Zephyr/modules/hal/stm32;D:/Zephyr/modules/hal/cmsis_6;D:/Zephyr/modules/hal/cmsis" CACHE STRING "")
set(ZEPHYR_SDK_INSTALL_DIR "D:/Zephyr/zephyr-sdk-0.16.8" CACHE PATH "")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_project)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE src)
target_compile_options(app PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>)

# CMSIS: external headers (core_cm4.h etc) and project wrapper (cmsis_core.h)
zephyr_include_directories("D:/Zephyr/modules/hal/cmsis/CMSIS/Core/Include")
zephyr_include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
zephyr_compile_definitions(__PROGRAM_START)

add_subdirectory(drivers)
add_subdirectory(algorithm)
add_subdirectory(modules)
add_subdirectory(topic)
add_subdirectory(cmd/shell)

if(CONFIG_${CONFIG_SYM})
  target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/apps/System_startup.cpp
  )
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/apps
  )
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/thread)
endif()
```

### 修改点说明

| 行 | 修改 | 说明 |
|----|------|------|
| 33 | `ZEPHYR_MODULES` 追加 STM32 和 CMSIS 模块路径 | 原为 `${SDK_GLUE_DIR}` 只有 sdk_glue |
| 34 | `ZEPHYR_SDK_INSTALL_DIR` 改为 0.16.8 | SDK 降级 |
| 42 | 新增 `zephyr_include_directories("...cmsis/CMSIS/Core/Include")` | 外部 CMSIS 模块的真实头文件路径（`core_cm4.h`、`cmsis_gcc.h` 等） |
| 43 | 新增 `zephyr_include_directories("...include")` | 项目自带的 `cmsis_core.h` wrapper |
| 44 | 新增 `zephyr_compile_definitions(__PROGRAM_START)` | CMSIS Core 子目录本应加的宏 |

### 为什么不需要更多 STM32Cube HAL 补丁

v4.3 的 `hal_stm32` 模块——它的 CMakeLists.txt 同样有 `add_subdirectory_ifdef(CONFIG_HAS_STM32CUBE stm32cube)` 时序问题，但 v4.3 的 `stm32cube` 子目录在**首次 cmake 配置后**，`CONFIG_HAS_STM32CUBE` 被缓存到 CMakeCache 中。第二次配置（`west build` 不带 `-p`）时该条件为真，子目录正常加载。

首次构建（`-p`）时虽然子目录不加载，但编译器只缺少 `SystemCoreClock` 等链接符号，这些在首次 `-p` 构建中会因为 Kconfig 处理后有了缓存值，第二次构建（不带 `-p`）时补全。但本项目每次均使用 `-p`，所以需要额外添加 `include` 路径和 `__PROGRAM_START` 宏。

不过 `-DSTM32F407xx`、`-DUSE_HAL_DRIVER` 等编译定义来自 Zephyr SOC 目录的 cmake（`soc/st/stm32/stm32f4x/`），它在 Kconfig 处理后的阶段（`zephyr_default.cmake` 第 10 位 `soc` 模块）加载，因此不受此时序问题影响。

---

## 4. Kconfig 修改

文件：`E:\Zephyr_project\HPMicro\blinky\Kconfig`

### 修改内容

在 `source "Kconfig.zephyr"` 前新增：

```kconfig
config HAS_CMSIS_CORE
    bool

source "Kconfig.zephyr"
```

### 原因

`modules/Kconfig` 中存在历史遗留的 `if 0` 块（v3.7 之前用于兼容旧版模块结构）：

```kconfig
if 0
osource "modules/*/Kconfig"
endif
```

Zephyr 的 kconfiglib 实现在 `if 0` 内仍然会进入文件并解析符号（只是设依赖为 0）。`modules/cmsis/Kconfig` 定义：

```kconfig
config HAS_CMSIS_CORE
    bool
    select HAS_CMSIS_CORE_A if CPU_AARCH32_CORTEX_A
    select HAS_CMSIS_CORE_R if CPU_AARCH32_CORTEX_R
```

此定义在 `if 0` 内，因此 `HAS_CMSIS_CORE` 的依赖为 `n`。

而 `modules/Kconfig.stm32` 中（不在 `if 0` 内）：

```kconfig
config HAS_STM32CUBE
    bool
    select HAS_CMSIS_CORE
    depends on SOC_FAMILY_STM32
```

`HAS_STM32CUBE` 对依赖为 `n` 的 `HAS_CMSIS_CORE` 执行 `select y`。Kconfig 认为这是错误，而 Zephyr 的 kconfig.py 将所有 Kconfig warning 视为 error，阻断编译。

在项目 Kconfig 中预定义 `HAS_CMSIS_CORE`（在 `source "Kconfig.zephyr"` 外侧）使得其联合依赖变为 `y || 0 = y`，消除冲突。

v4.3 和 v4.4 都有此 `if 0` 块，因此两个版本都需要这个修补。

---

## 5. 新增文件：`include/cmsis_core.h`

文件：`E:\Zephyr_project\HPMicro\blinky\include\cmsis_core.h`

### 内容

```c
/* CMSIS core header for Cortex-M — shadows Zephyr's empty stub */
#if defined(CONFIG_CPU_CORTEX_M)
/* CMSIS v5.7+ renamed SHPR to SHP; Zephyr v4.3 still uses old name */
#define SHPR SHP
#include <soc.h>
#include <core_cm4.h>
#elif defined(CONFIG_CPU_AARCH32_CORTEX_A) || defined(CONFIG_CPU_AARCH32_CORTEX_R)
/* Zephyr's internal module handles these */
#endif
```

### 作用

Zephyr v4.3 内部 `modules/cmsis/cmsis_core.h` 内容：

```c
#if defined(CONFIG_CPU_AARCH32_CORTEX_A) || defined(CONFIG_CPU_AARCH32_CORTEX_R)
#include "cmsis_core_a_r.h"
#endif
```

即对 Cortex-M 完全为空。但 ARM AArch32 架构代码（`include/zephyr/arch/arm/asm_inline_gcc.h`）中无条件：

```c
#include <cmsis_core.h>
```

并直接使用 CMSIS 函数（`__get_BASEPRI`、`NVIC_SetPriority`）和寄存器定义（`SCB`、`NVIC_Type`）。这些定义来自标准 CMSIS 头文件 `core_cm4.h`，该文件通过 `soc.h` → `stm32f4xx.h` → `stm32f407xx.h` → `core_cm4.h` 链展开。

Zephyr v4.4 的 `modules/cmsis_6/cmsis_core_m.h` 中显式 `#include <soc.h>`，v4.3 无此文件。因此需要在项目级提供一个正确的 `cmsis_core.h`，在 Zephyr 的空 stub 之前被找到。

由于 `include/` 目录通过 `zephyr_include_directories` 加到全局包含路径最前面，`#include <cmsis_core.h>` 优先找到项目版本，而不是 Zephyr 内部的空 stub。

### 为什么 CMakeLists.txt 里还要加 DTC 等路径

STM32F4 的 DTC/SOC 路径（`stm32f4xx/soc`、`stm32f4xx/drivers/include`、`common_ll/include`）来自 `ZEPHYR_MODULES` 中注册的 `hal_stm32` 模块，该模块的 DTS root 注册在 CMakeLists.txt 第 19 行的 DTS_ROOT 中，在 DTS 处理阶段（`pre_dt.cmake`）被解析并加入编译器的包含路径。因此不需要在 CMakeLists.txt 中手动添加。

---

## 6. sdk_glue 修改

文件：`D:\Zephyr_HPMicro\sdk_glue\drivers\flash\flash_hpmicro.c`

### 修改内容

| 原文 | 改为 | 原因 |
|------|------|------|
| `PARTITION_OFFSET(boot_partition)` | `FIXED_PARTITION_OFFSET(boot_partition)` | Zephyr v4.3 中宏名变更 |
| `PARTITION_SIZE(...)` | `FIXED_PARTITION_SIZE(...)` | 同上 |

Zephyr 在某个版本中将 `PARTITION_OFFSET`/`PARTITION_SIZE` 重新命名为 `FIXED_PARTITION_OFFSET`/`FIXED_PARTITION_SIZE`，旧名在 v4.3 中已不存在。sdk_glue 0.7.0 仍使用旧名。

### 相关头文件

这些宏定义在 `include/zephyr/storage/flash_map.h` 或通过 devicetree `fixed-partitions` binding 生成。

---

## 7. ZEPHYR_MODULES 值说明

```cmake
set(ZEPHYR_MODULES "${SDK_GLUE_DIR};D:/Zephyr/modules/hal/stm32;D:/Zephyr/modules/hal/cmsis_6;D:/Zephyr/modules/hal/cmsis" CACHE STRING "")
```

各模块作用：

| 模块 | 路径 | 作用 |
|------|------|------|
| sdk_glue | `D:\Zephyr_HPMicro\sdk_glue` | HPMicro 板级/SOC/DTS 定义，driver |
| hal_stm32 | `D:\Zephyr\modules\hal\stm32` | STM32Cube HAL 源文件和头文件 |
| hal_cmsis | `D:\Zephyr\modules\hal\cmsis` | ARM CMSIS 标准头文件（core_cm4.h、cmsis_gcc.h 等） |
| hal_cmsis_6 | `D:\Zephyr\modules\hal\cmsis_6` | CMSIS v6，v4.3 manifest 中已存在，本次构建未直接使用 |

STM32 构建时依赖 `hal_stm32` 和 `hal_cmsis`，`hal_cmsis_6` 实际为空目录（`west update` 后无文件）。

HPM 构建时只依赖 `sdk_glue`，其余模块不会影响 RISC-V 编译（ARM 相关的头文件和宏不会在 RISC-V 工具链中展开）。

---

## 8. 构建测试

### STM32F4 Discovery

```powershell
west build -b stm32f4_disco -p
```

结果：通过，171 步全部完成。

```
Memory region         Used Size  Region Size  %age Used
           FLASH:       33740 B         1 MB      3.22%
             RAM:       14160 B       128 KB     10.80%
```

### HPM6E00 EVK

```powershell
west build -b hpm6e00evk -p
```

结果：通过（flash 驱动修后），应用代码编译成功。

---

## 9. 补充说明

### 9.1 v4.3.0 仍然存在的问题

- `modules/Kconfig` 中的 `if 0 osource "modules/*/Kconfig"` 仍然存在，但只影响 CMSIS 模块的 Kconfig，项目中已通过 Kconfig 预定义解决
- 模块 CMakeLists.txt 加载时序问题依然存在，但对 v4.3 影响范围更小（仅 CMSIS Core 子目录），已通过 `include/cmsis_core.h` 绕过
- `asm_inline_gcc.h` 无条件 `#include <cmsis_core.h>`——v4.4 同样有此包含

### 9.2 多板切换注意事项

CMakeLists.txt 中的 `zephyr_include_directories` 等设置是**全局生效**的。STM32 构建需要 CMSIS 路径，HPM 构建不需要但也不受影响（RISC-V 编译链不解析 ARM CMSIS 头文件）。如需按条件隔离，可加 `if(CONFIG_SOC_FAMILY_STM32)` 包裹。

### 9.3 从零复现

```powershell
cd D:\Zephyr
west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.3.0
west update

# 下载 SDK 0.16.8 解压到 D:\Zephyr\zephyr-sdk-0.16.8
# 下载 sdk_glue 到 D:\Zephyr_HPMicro\sdk_glue
# 拉取本项目到 E:\Zephyr_project\HPMicro\blinky

cd E:\Zephyr_project\HPMicro\blinky
$env:ZEPHYR_BASE = 'D:\Zephyr\zephyr'
west build -b stm32f4_disco -p
west build -b hpm6e00evk -p
```
