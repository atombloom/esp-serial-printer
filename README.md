# esp-serial-printer

基于 ESP-IDF 的 **AtomBlool / MTP02 串口热敏打印机** 对接组件：通过 UART 按固定帧格式下发控制指令与 384×N 点阵行数据，并做与上位机脚本一致的缓冲流控。

## 功能概览

- 读打印头温度（命令 `0x06`，协议见文档）
- 电机走纸 / 停止（`0x04` / `0x05`）
- 多行位图打印：`PrintRasterRows()` 两阶段预送、`0x0E` 方向、`0x02` 开始打印、应答 `255` 重试、轮询 `0x09` 剩余行数，可选结束后 ±300 步出纸（与 `scripts/serial_print_core.py` 对齐）
- **满幅固定尺寸位图**：`PrintFixedRasterImage()`，缓冲区须恰好 `EspSerialPrinter::kFixedRasterImageBytes`（384 行 × 48 字节/行 = 18432 字节），行优先排列；仓库内示例数据见 `images/pic_chessboard.h`、`images/pic_lanyangyang.h`
- 内置条纹测试图：`PrintBuiltInTestPattern()`，数据在 `images/esp_serial_printer_test_image.h`
- 可选后台打印：`StartPrintWorker()` + `EnqueuePrintRasterRowsAsync()` / `EnqueuePrintFixedRasterImageAsync()` / `EnqueuePrintBuiltInTestPatternAsync()`（不阻塞调用方）

## 目录结构

| 路径 | 说明 |
|------|------|
| `esp_serial_printer.h` / `esp_serial_printer.cc` | 打印机类 `EspSerialPrinter` |
| `esp_serial_printer_protocol.h` | 6 字节控制帧、51 字节行帧、组包与解析 |
| `images/esp_serial_printer_test_image.h` | 内置条纹测试位图（16 行 × 48 字节/行） |
| `images/pic_chessboard.h` | 示例满幅图：`chessboard_mask[18432]`，`CHESSBOARD_MASK_LINES` = 384 |
| `images/pic_lanyangyang.h` | 示例满幅图：`lanyangyang_mask[18432]`，`LANYANGYANG_MASK_LINES` = 384 |
| `docs/串口上位机打印协议.md` | 与下位机一致的协议说明 |
| `docs/实现需求.md` | 组件能力与设计约束 |
| `scripts/serial_print_core.py` / `print_image_terminal.py` | PC 端参考实现（需同目录下其它 Python 依赖时自行补齐） |

## 依赖与接入

- **CMake**：`REQUIRES driver`（延时使用 FreeRTOS `vTaskDelay`，不依赖 `main/common/utility`）。
- 在本仓库中可将组件以本地路径依赖加入应用的 `idf_component.yml`，例如：

```yaml
dependencies:
  esp-serial-printer:
    path: ../components_my/esp-serial-printer
```

- 若应用 `CMakeLists.txt` 未自动传递依赖，请在 `main` 的 `idf_component_register` 中增加对 `esp_serial_printer` 的 `REQUIRES`（名称以 IDF 生成的组件名为准，一般为下划线形式）。
- 应用侧若 `#include "images/pic_xxx.h"`，需保证编译单元能解析到组件内 `images/`（本仓库 `main/tests/test_app_printer.cc` 通过 `main` 的 `INCLUDE_DIRS` 等方式包含组件头路径，按你工程实际配置即可）。

## 位图与打印图片约定

- **物理宽度**：384 点/行；协议每行 **48 字节**（MSB 先、1 表示打印点），与 `EspSerialPrinter::kRowBytes` 一致。
- **两种调用方式**：
  1. **`PrintFixedRasterImage(raster, print_direction, auto_feed)`**  
     `raster` 必须指向连续 **18432** 字节（`kFixedRasterImageBytes`），即 **384 行** 的满幅图。组件内部等价于 `PrintRasterRows(raster, kFixedRasterImageRows, …)`。
  2. **`PrintRasterRows(row_data, row_count, print_direction, auto_feed)`**  
     行数可小于 384（例如短条、内置测试图逻辑）；`row_data` 长度为 `row_count * 48`。
- **打印方向**：`0` 正常 / `1` 反方向，与 Python `DEFAULT_PRINT_DIRECTION` 一致；可直接使用 `EspSerialPrinter::kDefaultPrintDirection`（值为 `1`）。
- **`auto_feed`**：为 `true` 时在整图打印结束后按方向自动走纸约 ±300 步（与 PC 脚本一致）；不需要出纸可传 `false`。
- **自制图片**：将图像转为上述行优先、每行 48 字节的 C 数组即可；生成流程可与 `scripts/serial_print_core.py` 对齐。

## 使用示例（与 `main/tests/test_app_printer.cc` 联调流程一致）

以下为应用内联调常用顺序：**UART 初始化 → 可选读温度 → 打印满幅位图**。串口端口与 TX/RX GPIO **必须按实际硬件修改**，且勿与控制台 UART（常见为 `UART_NUM_0`）冲突。

```cpp
#include "esp_serial_printer.h"
#include "images/pic_lanyangyang.h"  // 或 pic_chessboard.h：chessboard_mask

// 按板子接线修改（示例同 test_app_printer.cc）
constexpr uart_port_t kPrinterUart = UART_NUM_1;
constexpr gpio_num_t kPrinterTxPin = GPIO_NUM_8;
constexpr gpio_num_t kPrinterRxPin = GPIO_NUM_9;

// 阻塞方式打印
void RunPrinterWithImage() {
    EspSerialPrinter printer(kPrinterUart, kPrinterTxPin, kPrinterRxPin);

    esp_err_t err = printer.Init();
    if (err != ESP_OK) {
        return;
    }

    int16_t temperature_raw = 0;
    float temperature_celsius = 0.F;
    err = printer.ReadDeviceTemperature(&temperature_raw, &temperature_celsius);
    // 联调时可打日志；失败不阻止后续打印

    // 打印满幅图：lanyangyang_mask / chessboard_mask 均为 18432 字节
    err = printer.PrintFixedRasterImage(
        lanyangyang_mask,
        EspSerialPrinter::kDefaultPrintDirection,
        /*auto_feed*/ true);
    if (err != ESP_OK) {
        return;
    }
}

// 非阻塞方式打印
void RunPrinterWithImageUnblock() {
    EspSerialPrinter printer(kPrinterUart, kPrinterTxPin, kPrinterRxPin);

    esp_err_t err = printer.Init();
    if (err != ESP_OK) {
        return;
    }

    int16_t temperature_raw = 0;
    float temperature_celsius = 0.F;
    err = printer.ReadDeviceTemperature(&temperature_raw, &temperature_celsius);
    // 联调时可打日志；失败不阻止后续打印

    // 专用后台任务持锁执行 PrintRasterRowsUnlocked；此处仅创建任务与入队，当前调用栈不阻塞整图时长
    err = printer.StartPrintWorker();
    if (err != ESP_OK) {
        ESP_LOGE(kLogTag, "StartPrintWorker failed: %s", esp_err_to_name(err));
        return;
    }

    err = printer.EnqueuePrintFixedRasterImageAsync(vectory_mask , EspSerialPrinter::kDefaultPrintDirection, true);
    if (err != ESP_OK) {
        ESP_LOGE(kLogTag, "EnqueuePrintFixedRasterImageAsync failed: %s", esp_err_to_name(err));
        return;
    }
}
```

```

**切换示例图**：将 `lanyangyang_mask` 换成 `chessboard_mask` 并包含 `pic_chessboard.h` 即可。

**非满幅或内存中的缓冲区**（行数 ≠ 384）：

```cpp
// 假设 my_rows 指向 N 行数据，每行 48 字节，总长 N * 48
ESP_ERROR_CHECK(printer.PrintRasterRows(my_rows, row_count,
                                        EspSerialPrinter::kDefaultPrintDirection,
                                        true));
```

**内置短测试图**（无需自备 18432 字节数组）：

```cpp
ESP_ERROR_CHECK(printer.PrintBuiltInTestPattern(EspSerialPrinter::kDefaultPrintDirection, true));
```

## 其它初始化与流控

```cpp
EspSerialPrinter printer(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18);
// 默认 Init() 会在 UART 就绪后发送 0x0E 打印方向（默认 1，与 Python DEFAULT_PRINT_DIRECTION 一致）
ESP_ERROR_CHECK(printer.Init());
// 不需要下发方向时可：EspSerialPrinterInitConfig cfg = EspSerialPrinter::DefaultInitConfig();
// cfg.apply_print_direction = false; ESP_ERROR_CHECK(printer.Init(&cfg));
```

流控参数可通过 `EspSerialPrinterFlowOptions` + `SetFlowOptions()` 调整（阶段 1 行间隔、缓冲满重试次数与间隔、读应答超时、打印结束后 settle 时间等）。

**初始化与 Python 脚本的关系**：`serial_print_core.py` 在每次打印流程的阶段 1 之后、`START_PRINT` 之前发送 `SET_PRINT_DIR`。本组件在 `Init()` 末尾可选先同步一次方向（默认开启）；`PrintRasterRows()` / `PrintFixedRasterImage()` 仍会在开始正式打印前再次调用 `SetPrintDirection()`，与现网脚本一致。也可单独调用 `SetPrintDirection()` 或设置 `EspSerialPrinterInitConfig::send_stop_print_on_init` 在上电时先发 `0x03` 清空缓存。

## 串口参数

与协议文档一致：**115200，8N1**，控制帧与数据帧格式见 [docs/串口上位机打印协议.md](./docs/串口上位机打印协议.md)。

## 代码约定

- 协议常量与组包放在 `esp_serial_printer_protocol.h`，与设备逻辑解耦，便于对照文档维护。
- 注释与命名以保持可读为主，避免过短标识符。
