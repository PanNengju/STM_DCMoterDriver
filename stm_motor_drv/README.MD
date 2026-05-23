# DC Motor Driver — STM32 直流电机驱动固件

基于 FreeRTOS 的直流有刷/无刷电机 FOC 控制固件，集成 CANopen (CiA 301 + CiA 402) 运动控制协议栈。

## 硬件平台

| 项目 | STM32F103C8 (当前) | STM32F407VG (可选) |
|------|-------------------|-------------------|
| 内核 | Cortex-M3 / 72MHz | Cortex-M4F / 168MHz |
| Flash | 64 KB | 512 KB |
| RAM | 20 KB | 128 KB |
| PWM | TIM1, 互补输出+死区 | TIM1, 互补输出+死区 |
| 编码器 | TIM2 (编码器模式) | TIM2 (编码器模式) |
| ADC | ADC1 + DMA | ADC1 + DMA |
| CAN | CAN1, 1Mbps | CAN1, 1Mbps |
| 调试 | USART2, 115200 | USART2, 115200 |
| 看门狗 | IWDG | WWDG |

## 构建

```bash
# 唯一外部依赖: arm-none-eabi-gcc
# 所有库 (STM32 HAL / CMSIS / FreeRTOS) 已内置于项目目录

make PLATFORM=F1          # STM32F103C8 (默认)
make PLATFORM=F4          # STM32F407VG
make clean                # 清理
```

**构建产出** (位于 `build/`):
- `motor_drv.elf` — ELF (含调试信息)
- `motor_drv.bin` — 二进制固件
- `motor_drv.hex` — Intel HEX

**资源占用** (F103, -Os, 含 FreeRTOS + HAL + CANopen):

| 段 | 大小 | 占比 |
|----|------|------|
| Flash (text) | ~36 KB | 55% |
| RAM (data+bss) | ~15 KB | 74% |

### 烧录

```bash
make flash    # OpenOCD + ST-Link
```

### 调试

```bash
make debug    # arm-none-eabi-gdb → :3333 (需先启动 OpenOCD 服务)
```

## 架构

```
main.c ── 系统时钟 / 外设初始化 / 任务创建 / 异常处理
  │
  ├─ TIM3 ISR (20kHz) ──→ 电流环 PID + FOC
  │
  ├─ SpeedLoop 任务 (1kHz)  ──→ 速度环 PID + 轨迹生成
  ├─ PosLoop 任务 (200Hz)    ──→ 位置环 PID + 位置序列
  ├─ CANopen 任务            ──→ NMT / SDO / PDO / EMCY / HB
  ├─ UartCLI 任务            ──→ 调试命令行 + Trace 实时数据
  └─ Monitor 任务 (100Hz)    ──→ 保护监测 / LED / 看门狗喂狗
```

### 控制环路

| 环路 | 频率 | 实现方式 | 说明 |
|------|------|---------|------|
| 电流环 | 20kHz | TIM3 硬件中断 | 最高优先级, FOC 计算 |
| 速度环 | 1kHz | FreeRTOS Task | 速度 PID + 加速度 Profile |
| 位置环 | 200Hz | FreeRTOS Task | 位置 PID + 位置序列 |
| 监测 | 100Hz | FreeRTOS Task | 保护、状态汇报 |

### CANopen 协议

| 服务 | COB-ID | 说明 |
|------|--------|------|
| NMT | 0x000 | 网络管理 (启停/复位/状态切换) |
| SDO | 0x600+ID / 0x580+ID | 服务数据对象 (参数读写) |
| RPDO1 | 0x200+ID | 控制字 + 目标值1 |
| RPDO2 | 0x300+ID | 运行模式 + 目标值2 |
| TPDO1 | 0x180+ID | 状态字 + 实际速度 |
| TPDO2 | 0x280+ID | 实际位置 + 实际电流 |
| TPDO3 | 0x380+ID | 实际转矩 + 温度 + 母线电压 |
| EMCY | 0x080+ID | 紧急报文 |
| Heartbeat | 0x700+ID | 心跳 (周期可配) |

支持运行模式: 位置模式 (PP)、速度模式 (PV)、转矩模式 (TQ)、回零模式 (HM)。

### 保护功能

- 过流/过载检测 (带时间累积阈值)
- 堵转检测 (电流阈值 × 时间窗口)
- 过温 (驱动器 + 电机双通道 NTC)
- 过压/欠压
- 编码器断线检测
- CAN 通信超时 (Heartbeat Consumer)
- 急停 (外部中断, 硬件刹车)
- 限位开关 (双向)
- IWDG 独立看门狗 (~1s 超时)
- 硬件故障 (HardFault/MemManage/BusFault → 紧急 PWM 停止)

## 文件结构

```
stm_motor_drv/
├── Makefile                  # 构建脚本
├── STM32F103C8Tx_FLASH.ld    # 链接脚本
├── inc/
│   ├── motor_config.h        # 平台/参数/类型定义 (所有模块共享)
│   ├── motor_drv.h           # 顶层模块聚合
│   ├── FreeRTOSConfig.h      # FreeRTOS 内核配置
│   ├── stm32f1xx_hal_conf.h  # STM32 HAL 模块使能与时钟配置
│   ├── pid.h / pid.c         # PID 控制器
│   ├── pwm.h / pwm.c         # PWM 死区互补输出
│   ├── encoder.h / encoder.c # 编码器 (M/T 法测速)
│   ├── adc.h / adc.c         # ADC+DMA 多通道采样
│   ├── protection.h          # 保护检测
│   ├── flash.h / flash.c     # Flash 参数存储 (磨损均衡)
│   ├── state_machine.h       # CIA 402 状态机
│   ├── uart_cli.h            # UART 调试 CLI
│   └── canopen/              # CANopen 协议栈 (CiA 301 + 402)
│       ├── canopen.h         #   主模块
│       ├── co_nmt.h          #   网络管理
│       ├── co_sdo.h          #   服务数据对象
│       ├── co_pdo.h          #   过程数据对象
│       ├── co_emcy.h         #   紧急报文
│       ├── co_sync.h         #   同步报文
│       ├── co_cia402.h       #   CiA 402 运动控制
│       └── co_objdict.h      #   对象字典
├── src/                      # 对应 .c 实现文件
├── STM32_HAL/                # STM32 HAL + CMSIS (自包含)
└── FreeRTOSv202212.01/       # FreeRTOS v10.5.1 (自包含)
```

## UART CLI 命令

| 命令 | 说明 |
|------|------|
| `state` | 查看系统状态、故障码 |
| `speed <rpm>` | 设置目标转速 |
| `pos <pulses> [rel]` | 设置目标位置 (可选相对) |
| `torque <nm_x100>` | 设置目标转矩 |
| `stop [coast\|brake]` | 停止电机 |
| `estop` | 紧急停止 |
| `reset` | 清除故障 |
| `zero [manual\|auto]` | 传感器调零 |
| `pid show` | 查看当前 PID 参数 |
| `pid set <loop> <kp> <ki> <kd>` | 设置 PID 参数 |
| `params` | 查看电机参数 |
| `log <off\|error\|warn\|info\|debug>` | 设置日志等级 |
| `trace <on\|off>` | 实时数据流开关 |
| `save` | 保存参数到 Flash |
| `factory` | 恢复出厂设置 |
| `help` | 显示所有命令 |

## 引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| PWM-A (H桥正) | PA8 | TIM1_CH1 + CH1N (PB13) |
| PWM-B (H桥负) | PA9 | TIM1_CH2 + CH2N (PB14) |
| 编码器 A | PA0 | TIM2_CH1 |
| 编码器 B | PA1 | TIM2_CH2 |
| 电流采样 | PA0 | ADC1_IN0 |
| 电压采样 | PA1 | ADC1_IN1 |
| 驱动器温度 | PA2 | ADC1_IN2 |
| 电机温度 | PA3 | ADC1_IN3 |
| CAN RX | PB8 | CAN1_RX (AFIO Remap) |
| CAN TX | PB9 | CAN1_TX (AFIO Remap) |
| UART TX | PA2 | USART2_TX |
| UART RX | PA3 | USART2_RX |
| 急停 | PB0 | EXTI0 (低电平触发) |
| 限位- | PB12 | GPIO 输入 |
| 限位+ | PB13 | GPIO 输入 |
| LED 绿 | PC13 | 运行指示 |
| LED 黄 | PC14 | 警告指示 |
| LED 红 | PC15 | 故障指示 |

## 许可证

MIT License
