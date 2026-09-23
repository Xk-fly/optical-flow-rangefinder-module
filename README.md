# Optical Flow Rangefinder Module

基于STM32G030的无人机下视光流/测距模块固件：

- PMW3901光流，通过MAVLink 1 `OPTICAL_FLOW`（消息ID 100）约50Hz发送；
- VL53L1X测距，通过`DISTANCE_SENSOR`（消息ID 132）约10Hz发送；
- 串口：115200、8N1、3.3V TTL；
- 面向MatekF405 / ArduPilot 4.5.7使用。

## 测距滤波

VL53L1X读取路径使用轻量级三点中值滤波：

1. 仅接受50～3600mm范围内的数据；
2. 前两个有效样本直接输出；
3. 从第三个有效样本开始，输出最近三个有效样本的中值；
4. 最近一次有效样本超过300ms后，旧窗口自动清空；恢复后的第一帧直接采用新距离，不会被过期历史数据覆盖；
5. 同一个300ms常量也用于判断`OPTICAL_FLOW.ground_distance`是否新鲜，避免驱动滤波和MAVLink发送使用不同的过期标准。

该滤波主要抑制单帧跳变。连续两帧以上的异常值仍可能通过；稳定测量阶段会引入约一个采样周期（约100ms）的阶跃响应延迟。

## 地面近场启动（Ground Bootstrap）

模块安装后，VL53L1X镜头到起降平面的实际距离约2～3cm，而本项目保持VL53真实可信测距范围为50～3600mm。为避免地面近场无距离时ArduPilot无法建立光流相对水平位置，本分支加入受限的地面启动状态机：

1. VL53初始化必须成功；初始化失败时绝不发送虚拟距离；
2. 只有VL53完成真实读取且结果为“距离小于50mm”时，才发送50mm（5cm）的地面启动距离；
3. 50mm与VL53真实有效下限、MAVLink `DISTANCE_SENSOR.min_distance=5cm`、飞控 `RNGFND1_MIN_CM=5`保持一致；
4. 真实距离达到至少60mm并连续确认2次后，永久切换到真实距离模式；
5. 一旦进入真实距离模式，本次上电期间绝不退回虚拟距离；飞行中若VL53报错、过远或再次低于有效范围，会按原有freshness/timeout机制失效，而不会伪装成5cm；
6. `NOT_READY`不会被当成故障，也不会人工生成新的距离，只依赖最近一次数据的300ms freshness。

因此虚拟值只解决“传感器正常但起飞前距离低于其可靠量程”的启动问题，不用于掩盖I2C/API故障或飞行中的测距失效。地面启动阶段会发送一次 `VL53 ground bootstrap 5cm`，切入真实测距后会发送一次 `VL53 real range locked`，便于从飞控日志确认状态切换。

## 关键生产行为

- PMW3901初始化后丢弃首个Motion Burst样本；
- 保持已验证的光流X/Y轴及符号，不交换、不重复取反；
- 光流约50Hz，测距约10Hz，心跳约1Hz；
- 生产构建关闭周期性PMW原始诊断；
- VL53读取为非阻塞就绪检查，并在读取后清除传感器中断；
- 真实距离同时用于`OPTICAL_FLOW.ground_distance`和独立`DISTANCE_SENSOR`消息。

## 回归检查

```bash
python tools/verify_pmw3901_driver.py
python tools/verify_production_firmware.py
python tools/test_monitor_mavlink_serial.py
bash tools/test-vl53-median-filter.sh
bash tools/test-vl53-ground-bootstrap.sh
```

滤波核心是独立的纯C模块，测试脚本会从自身位置定位仓库根目录，并使用`CC`环境变量指定的主机编译器（默认`gcc`）。Windows可在任意带GCC的WSL发行版中进入仓库后执行同一脚本，不依赖固定盘符、目录名或发行版名称。

## Windows固件编译

项目优先复用已安装的STM32CubeCLT和STM32CubeIDE GNU Make：

```bash
BUILD_OUT=C:/Users/Public/stm32-f-flow-build bash tools/build-windows.sh
```

输出包括`.elf`、`.hex`和`.bin`。应使用项目外的clean build目录，避免旧目标文件污染固件。

## 本地交付记录（2026-09-18）

- 固件：`Build/F_FLOW_PRODUCTION_MEDIAN3_FRESHNESS.hex`
- HEX大小：68,514字节
- SHA-256：`e874f98bf24782ae2d189de4960ce354c141f9362ca60612edbf9f46ba12bc87`
- ELF占用：text 24,316字节，data 16字节，bss 2,464字节
- 验证：滤波核心主机测试、生产合同检查、PMW3901静态检查、MAVLink监听器离线测试及Windows外部clean build均通过。

以上仅为代码和离线构建验证，尚未替代真实VL53数据和飞控UART台架测试。

## 台架验证门禁

编译成功不等于已经通过硬件验证。首次烧录后应在拆桨/无执行机构条件下确认：

- `OPTICAL_FLOW`约50Hz且quality有效；
- `DISTANCE_SENSOR`约10Hz；
- 固定目标下距离跳变减少；
- 快速改变高度后约300ms内恢复到新距离；
- 光流方向保持：机体向右移动时`flow_x < 0`，向前移动时`flow_y > 0`。

完成无桨接收、方向、频率和距离验证后，再考虑启用EKF光流速度源。
