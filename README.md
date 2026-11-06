# OPS9 / XRobot

使用 XRobot 1.0.1、LibXR v2.0.1 和 FreeRTOS复刻的OPS9项目代码
本项目面向 STM32F103C8T6，Flash 64 KiB、RAM 20 KiB。

## 代码组织

| 路径 | 职责 |
| --- | --- |
| `Core/`、`ops9.ioc` | CubeMX 外设、DMA、中断和 FreeRTOS 启动 |
| `User/app_main.cpp` | LibXR 生成的硬件对象和 XRobot 入口；业务接线在 User Code 区域 |
| `User/xrobot.yaml` | XRobot 生成的编译期参数 |
| `User/board/pulse_counter.hpp` | STM32 外部脉冲计数适配；HAL 依赖限定在此 |
| `User/sensors/hwt101.hpp` | HWT101 流式拆包、校验和与航向角解析 |
| `User/odometry/odometry.hpp` | 脉冲位移、方向映射、航向连续化、世界坐标积分 |
| `User/communication/protocol.hpp` | 两字节命令解析、14 字节位置报文和 CRC8 |
| `User/application/` | LibXR 周期线程、IMU 清零状态机、串口与 LED 调度 |
| `tests/` | 可在电脑上运行的协议、IMU、里程计算法测试 |

项目业务类保留在 BSP 的 `User/` 下，通过生成入口的 User Code 区域装配，
使用 `LibXR::UART`、`GPIO`、`PowerManager` 和 `Thread`。
`Modules/modules.yaml` 暂无外部模块依赖，不需要额外发布或克隆 OPS9 模块仓库。
`XROBOT_MAIN()` 运行框架监控循环，OPS9 工作由独立的 LibXR 线程执行。

## 硬件与默认参数

| 功能 | 外设 / 引脚 |
| --- | --- |
| 编码器 1 脉冲 / 方向 | TIM1 CH1 / PA8；PA10 |
| 编码器 2 脉冲 / 方向 | TIM2 CH1 / PA0；PA11 |
| IMU | USART1，PB6 TX / PB7 RX，115200 8N1 |
| 上位机 | USART2，PA2 TX / PA3 RX，115200 8N1 |
| LED | PC13 |
| HAL / LibXR 时间基准 | TIM4，1 ms 中断，优先级 0 |

TIM1/2 沿用单脉冲输入加独立方向脚的外部计数模式，不配置为 AB 相正交编码器。
方向脚为高电平时计数为正。每圈 1024 脉冲，轮半径 0.029 m，默认安装方向为 2。
PA11 在旧 `.ioc` 中重复标为 DIRT1，目标 BSP 使用 DIRT2，与旧代码读取的 PA11 一致。
预留的 STEP_ZERO1、STEP_ZERO2、ANGLE_ZERO 和 I2C 保留配置，未赋予新功能。

两个 UART 都由 LibXR 管理，RX 使用循环 DMA，TX 使用普通 DMA。
USART1 使用 DMA1 通道 5 RX / 4 TX；USART2 使用通道 6 RX / 7 TX；
相关 DMA 和 UART 中断优先级均为 5。

## 运行与协议

- 每 1 ms 排空串口接收队列并采样脉冲。位移直接由脉冲增量积分，不假设实际调度时间始终为 1 ms。
- 启动时发送 IMU 解锁，等待 20 ms，再发送清零指令并检查新收到的航向帧；最多等待 2 秒后进入运行。
  IMU 缺失不会无限阻塞启动；无有效航向时使用 0，通信中断时保持最后航向。
  启动清零期间的编码器脉冲被丢弃，与原程序先清零再开始主循环的用途一致。
- IMU 正常时 LED 每 1000 ms 翻转；超过 500 ms 未收到有效航向帧时每 100 ms 翻转。
- 每 2 ms 上报一次位置。115200 8N1 下，14 字节需要约 1.22 ms，不能稳定执行旧程序名义上的 1 ms 上报。
  队列拒收时丢弃本次快照，不累计过期位置。
- 上报格式：`5C` + `float32 x` + `float32 y` + `float32 yaw` + `CRC8`。
  浮点数为小端 IEEE754，x/y 单位 m，yaw 单位 rad。CRC8 初值 FF、反射多项式 8C，覆盖前 13 字节，与原项目一致。
- 接收格式：`C5 22` 软件复位；`C5 30` 到 `C5 33` 设置方向 0 到 3。
  完整的两个字节到齐即执行，非法命令忽略，连续 C5 用最后一个重新同步。
- 方向映射保持旧算法：0 为 `(-wheel2, wheel1)`，1 为 `(wheel1, wheel2)`，
  2 为 `(wheel2, -wheel1)`，3 为 `(-wheel1, -wheel2)`，再按 IMU 航向旋转到世界坐标。


## 构建

依赖：Python、CMake、Ninja、支持 C++20 的 arm-none-eabi GCC，以及 STM32Cube_FW_F1_V1.8.7。
当前 CubeMX 工程引用本机 `STM32Cube_FW_F1_V1.8.7`，
迁移到其他电脑时在 CubeMX 中配置固件库位置并重新生成。

```powershell
python -m pip install -r requirements.txt
git submodule update --init --recursive
xrobot setup --frozen
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

输出为 `build/Debug/ops9.elf`、`.hex`、`.bin`，或对应的 `build/Release/` 文件。
修改 `User/xrobot.yaml` 后运行 `xrobot gen` 再编译。

修改 `.ioc` 后先由 CubeMX 生成 CMake 工程，再运行：

```powershell
libxr stm32 setup -d . --xrobot
xrobot gen
cmake --preset Debug
cmake --build --preset Debug
```

保留 `Core/Src/freertos.c` 的 User Code 区域：默认任务调用 `app_main()`。
保持默认任务和 OPS9 线程各 2048 字节栈、FreeRTOS 堆 10240 字节，已启用栈溢出检测。
新增功能或模块后需重新核查堆余量和任务栈高水位。

## 测试与验证范围

已验证 Debug / Release 链接成功，以及 IMU 分段输入、坏校验、截断后恢复、错误帧类型、
正负航向、跨 ±π 连续化、四种安装方向、旋转后的位移、分批脉冲积分、命令同步和原协议 CRC8 黄金报文。
另已检查 CubeMX 与 LibXR 再生成后的业务接线保留和重新编译。

尚未烧录或进行板上验收。实际编码器方向与脉冲比例、HWT101 清零响应、串口连续上报、
复位命令及长时间运行时的堆和栈余量，需要连接实际硬件后验证。
