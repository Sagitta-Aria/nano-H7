# 第三方来源

- `firmware/vendor/CMSIS`：来自用户提供的 `newstm32H743iit6` 资料包内“实验0 Template工程模板-新建工程使用”。保留 ARM/ST 原版权与许可头。未修改这些头文件与 `system_stm32h7xx.c`。
- `firmware/startup_stm32h743xx.s`：复制自同一模板。MSP 栈改为 4096 字节、C 库 heap 保持 0；动态任务内存使用 FreeRTOS heap_4。保留原文件头。
- `firmware/vendor/FreeRTOS`：官方 `https://github.com/FreeRTOS/FreeRTOS-Kernel`，tag `V11.2.0`。仅纳入实际使用的内核源码、公共头、RVDS ARM_CM4F 端口和 heap_4；MIT 许可证位于其 `LICENSE.md`。
- Keil 工程 XML 根据板卡模板生成，源文件列表、包含路径、MicroLIB 与 scatter 改为本项目配置。
- GUI 运行时依赖 pyserial 3.x（BSD）；Tkinter 来自 Python 标准发行版。
- 测试依赖 Unicorn 2.1.4 与 pyelftools 0.32，只安装在被忽略的 `.testdeps`，不进入 MCU 固件或 GUI 运行依赖。
