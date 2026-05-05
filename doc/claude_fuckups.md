# Claude 蠢事记录

本文件记录 Claude 在 HPM6E80 → HPM5361 移植过程中犯的所有错误。

---

## 1. 创建多余文件

### 1.1 根目录 boards 文件夹

在 `e:\Zephyr_project\HPMicro\blinky\boards\` 创建了空目录。项目本来没有这个目录，创建它毫无意义。

### 1.2 project 目录放 board 文件

在 `projects/boards/` 下创建了 `board_5361.overlay`。LED 节点应该直接写进 sdk_glue 的 board DTS。project 目录不该放 board 定义文件。

### 1.3 单独 build 目录

用 `-d build_5361` 创建独立构建目录。应该直接用默认 `build` 目录。

---

## 2. Kconfig 问题

### 2.1 乱改 prj.conf

动了不应该动的 `prj.conf`。项目配置文件不是给 board 配置用的。

### 2.2 不改底层 Kconfig

把 `TRD_CHASSIS` 设成 `default n` 就算完事，没检查 `topic/Kconfig` 和 `cmd/shell/Kconfig` 里的配置项各自有 `default y`。

### 2.3 Kconfig.board 命名错误

把 board Kconfig 文件命名为 `Kconfig.board_hpm5361evk`，Zephyr HWMv2 只识别 `Kconfig.<board_name>` 模式。导致 `BOARD_HPM5361EVK` 未定义，构建直接炸了。这是 rename 之后引入的 bug。

---

## 3. DTS 问题

### 3.1 想在 overlay 放 flash 分区

flash 分区必须放在 board 主 DTS 里，overlay 处理时机晚于驱动初始化。

### 3.2 没检查现有 DTSI

`hpm53xx.dtsi` 里 PLIC/PLIC_SW/MTIMER 的 `interrupts-extended` 用了 IRQ 0，直到 `gen_isr_tables.py` 报错才查出来。

---

## 4. 代码问题

### 4.1 soc.c 时钟 API 参数

照搬 HPM6200 的 `sysctl_config_cpu0_domain_clock` 调用，传了 5 个参数。HPM5361 只有 4 个。

### 4.2 soc.c PMA API 误用

用了 `pma_config_attributes()`，但 HPM5361 不支持 PMA，需要改用 PMP。

### 4.3 board.c RESET_SOFT

直接写 `HPM_PPOR->RESET_SOFT = 1`，HPM5361 没有这个寄存器。

---

## 5. 流程问题

### 5.1 问不该问的问题

"这个也要移到 sdk_glue 里？" — 用户说过 project 目录不要放 board 文件，不需要问。

### 5.2 不检查目录结构

改之前没完整看项目目录结构，根目录的 `boards/` 空文件夹一直没发现。

### 5.3 改不该改的文件

多次改 `prj.conf`、`board_5361_defconfig` 等不该动的文件。

### 5.4 不留垃圾

旧板文件没及时删。

### 5.5 编译不过才想起来查

改完不上手验证，等 build 炸了才回来看。

---

## 6. 编译成功后的遗留问题

如以下问题仍然因为项目配置原因无法编译:
- `can` 相关驱动 Kconfig 默认会在没有 CAN 外设时也尝试启用
- `projects/thread/Kconfig` 中的 `default y` 需要改为 `default n` 以适配新芯片
