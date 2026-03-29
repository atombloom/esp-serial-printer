#ifndef _ESP_SERIAL_PRINTER_H_
#define _ESP_SERIAL_PRINTER_H_

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_err.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_serial_printer_protocol.h"

/**
 * @brief AtomBlool 串口热敏打印机（协议见 docs/串口上位机打印协议.md）
 *
 * 功能：读温度、电机走纸、按上位机脚本节奏发送位图并流控（缓冲满重试等）。
 * 阶段 2 行数据应答里的「缓冲行数」一般指下位机打印头侧行缓存占用；常见容量约 50 行，
 * 可用 EspSerialPrinterFlowOptions 按该容量调节节排，避免仅靠额外控制指令（如读温度）拖慢或超时。
 */
/**
 * @brief UART 就绪后可选下发的打印机配置（对齐 scripts/serial_print_core.py 常用默认值）
 *
 * 说明：Python 在每次 `run_serial_image_print` 里于阶段 1 之后、`START_PRINT` 之前发送
 * `SET_PRINT_DIR`；此处在 `Init()` 末尾可选先同步一次方向，便于上电后与下位机状态一致。
 * `PrintRasterRows()` 内仍会按参数再次发送 `0x0E`，与现网脚本行为一致。
 */
struct EspSerialPrinterInitConfig {
    /** 为 true 时在 Init 成功安装 UART 后发送命令 0x0E */
    bool apply_print_direction;
    /** 0 正常 / 1 反方向，与 DEFAULT_PRINT_DIRECTION=1 一致 */
    uint8_t print_direction;
    /** 为 true 时先发送 0x03 停止打印并清空缓存（协议：停止打印，清空打印缓存区） */
    bool send_stop_print_on_init;
};

struct EspSerialPrinterFlowOptions {
    /** 阶段 1 每行发送后的间隔（毫秒），与 serial_print_core 中 0.001s 对齐 */
    uint32_t phase1_row_delay_ms;
    /**
     * 阶段 2 每行成功发送后的基础间隔（毫秒）。打印头行缓存约数十行时，可再配合
     * phase2_high_print_head_buffer_rows / phase2_high_print_head_extra_delay_ms 在高占用时加节拍。
     */
    uint32_t phase2_row_delay_ms;
    /**
     * 行应答里打印头缓冲占用 ≥ 该阈值时追加节排（阶段 1、阶段 2 均生效，在各行基础 delay 之外）。
     * 打印头行缓存常见约 50 行；默认可为 18～26 等，0 表示关闭高水位节排。
     */
    uint32_t phase2_high_print_head_buffer_rows;
    /** 满足上述高水位时追加延时（毫秒），仅 vTaskDelay，不发额外控制帧 */
    uint32_t phase2_high_print_head_extra_delay_ms;
    /**
     * 应答为 255（缓冲满）时重试等待的基础毫秒数；实际等待 = base * (1 + 已重试次数)，上限见 esp_serial_printer.cc 内 cap，
     * 以便打印头慢消化时仍能推进，而非固定短间隔重试耗尽次数。
     */
    uint32_t buffer_full_retry_delay_ms;
    /** 阶段 2 单行在持续 255 时最多尝试次数（含首次发送） */
    unsigned buffer_full_max_attempts;
    /**
     * START_PRINT (0x02) 应答成功后，再延时这么久才进入阶段 2 发首行。
     * PC 脚本为 10ms；下位机开始走纸时若过短，易出现阶段 2 早期行应答长时间等不到（ESP_ERR_TIMEOUT）。
     */
    uint32_t after_start_print_delay_ms;
    /** 轮询「已保存数据量」为 0 时的间隔（毫秒） */
    uint32_t remain_poll_interval_ms;
    /** 全部行发送完后，按协议文档建议再等待打印机构完成的时间（毫秒）；0 表示不额外等待 */
    uint32_t settle_after_last_row_ms;
    /** 控制类命令（0x02/0x06/0x09 等）读 6 字节应答超时（毫秒） */
    uint32_t response_read_timeout_ms;
    /**
     * 行 mask 发出后等应答的超时（毫秒）。走纸繁忙时行应答往往比控制帧更慢；
     * 为 0 时与 response_read_timeout_ms 相同。
     */
    uint32_t row_response_timeout_ms;
    /**
     * 未在 row_response_timeout_ms 内收齐 6 字节行应答时，除首次外再整行重发次数（每次重发前 Drain RX）。
     * 脚本无此层；嵌入式默认可为 1，吸收偶发迟答，又远轻于多次重试造成的长时间阻塞。
     */
    unsigned row_read_timeout_extra_attempts;
    /** 行应答超时后、整行重发前等待（毫秒）；仅当 row_read_timeout_extra_attempts>0 时有意义 */
    uint32_t row_read_timeout_retry_delay_ms;
    /**
     * 行应答：RX 已对齐到 AA BB 后，至少再保留这么多毫秒用于读完余下 4 字节（截止时间取 max(原总截止, 此刻+本值)）。
     * 避免总超时大多耗在「等帧头」上导致尾字节读一半就超时；0 表示不延长。
     */
    uint32_t row_frame_tail_reserve_ms;
};

/**
 * @brief 异步/后台打印任务状态（供 UI 或业务轮询，避免阻塞主任务）
 */
enum class EspSerialPrinterJobState : uint8_t {
    /** 空闲：无后台任务在执行，可提交新异步任务（若队列未满） */
    kIdle = 0,
    /** 忙：后台打印任务正在执行（含 UART 独占阶段） */
    kBusy,
};

class EspSerialPrinter {
public:
    static constexpr uint32_t kDefaultUartBaudRate = 115200;
    static constexpr size_t kRowBytes = 48;
    /**
     * 固定尺寸一维位图：总长 18432 = 384 行 × kRowBytes（与 images/pic_chessboard.h 中 chessboard_mask 等一致）
     */
    static constexpr size_t kFixedRasterImageBytes = 18432;
    static constexpr size_t kFixedRasterImageRows = kFixedRasterImageBytes / kRowBytes;
    /** 与 serial_print_core.DEFAULT_PRINT_DIRECTION 一致 */
    static constexpr uint8_t kDefaultPrintDirection = 1;

    EspSerialPrinter(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin,
                     uint32_t baud_rate = kDefaultUartBaudRate);

    ~EspSerialPrinter();

    EspSerialPrinter(const EspSerialPrinter&) = delete;
    EspSerialPrinter& operator=(const EspSerialPrinter&) = delete;

    /**
     * @brief 安装 UART 驱动并配置 8N1（与协议一致：115200, 8N1）
     * @param init_config 若为 nullptr，使用默认：上电发送打印方向 0x0E=kDefaultPrintDirection，不发送停止打印
     */
    esp_err_t Init(const EspSerialPrinterInitConfig* init_config = nullptr);

    /** 默认 Init 行为：下发方向、不清缓存 */
    static EspSerialPrinterInitConfig DefaultInitConfig();

    /**
     * @brief 单独设置打印方向（命令 0x0E），与 Python send_cmd SET_PRINT_DIR 一致
     */
    esp_err_t SetPrintDirection(uint8_t print_direction);

    /**
     * @brief 设置流控与超时参数（可在 Init 之后、打印前调用）
     */
    void SetFlowOptions(const EspSerialPrinterFlowOptions& options);

    /**
     * @brief 读取温度（命令 0x06）
     * @param raw_out 原始 int16（协议：摄氏度×100，0xFFFF 异常）
     * @param celsius_out 若非空，输出换算后的摄氏度（兼容部分固件直接返回整型℃）
     */
    esp_err_t ReadDeviceTemperature(int16_t* raw_out, float* celsius_out);

    /**
     * @brief 电机走纸（命令 0x04，数据为步数 int16，正负表示方向）
     */
    esp_err_t FeedPaper(int16_t steps);

    /**
     * @brief 电机停止（命令 0x05）
     */
    esp_err_t StopMotor();

    /**
     * @brief 打印位图（多行，每行 48 字节），流程对齐 scripts/serial_print_core.py + 协议文档
     * @param row_data 行优先，总长 row_count * 48
     * @param print_direction 0 正常方向 / 1 反方向（命令 0x0E）
     * @param auto_feed 是否在结束后按方向自动出纸（与 Python 中 ±300 步一致）
     */
    esp_err_t PrintRasterRows(const uint8_t* row_data, size_t row_count, uint8_t print_direction,
                              bool auto_feed = true);

    /**
     * @brief 打印固定长度一维位图（恰好 kFixedRasterImageBytes 字节，行优先）
     * @param raster_image 指向 kFixedRasterImageBytes 字节的缓冲区（如 const uint8_t chessboard_mask[18432]）
     */
    esp_err_t PrintFixedRasterImage(const uint8_t* raster_image, uint8_t print_direction,
                                    bool auto_feed = true);

    /**
     * @brief 启动专用后台打印任务（FreeRTOS 任务内执行位图流程，不阻塞调用方）
     * @note 须先 Init() 成功；可重复调用，仅首次创建任务与队列
     * @param stack_words 任务栈（字），位图流程较深，建议 >= 4096
     * @param priority 任务优先级
     */
    esp_err_t StartPrintWorker(uint32_t stack_words = 4096, UBaseType_t priority = 5);

    /**
     * @brief 将位图打印请求放入后台队列（内部 malloc 拷贝 row_data，调用方可立即返回）
     * @return ESP_ERR_NO_MEM 分配失败；ESP_ERR_INVALID_STATE 未启动 worker 或忙/队列满
     */
    esp_err_t EnqueuePrintRasterRowsAsync(const uint8_t* row_data, size_t row_count, uint8_t print_direction,
                                          bool auto_feed = true);

    /**
     * @brief 后台队列打印固定 kFixedRasterImageBytes 字节一维位图（内部 malloc 拷贝）
     */
    esp_err_t EnqueuePrintFixedRasterImageAsync(const uint8_t* raster_image, uint8_t print_direction,
                                                bool auto_feed = true);

    /**
     * @brief 后台打印内置测试图（不拷贝大块缓冲区）
     */
    esp_err_t EnqueuePrintBuiltInTestPatternAsync(uint8_t print_direction, bool auto_feed = true);

    /**
     * @brief 打印会话状态：同步/异步任一路径在走 PrintRasterRows 流程时为 kBusy
     * @note 队列里尚有待处理任务但 worker 未开始执行时亦为 kBusy
     */
    EspSerialPrinterJobState GetPrintJobState() const;

    /**
     * @brief 最近一次后台打印结束时的 esp_err（成功为 ESP_OK）
     * @note 在 GetPrintJobState()==kIdle 且至少完成过一次后台任务后有参考意义
     */
    esp_err_t GetLastAsyncPrintResult() const;

private:
    uart_port_t uart_num_;
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    uint32_t baud_rate_;
    bool initialized_;
    EspSerialPrinterFlowOptions flow_;

    /** UART 会话互斥：同步打印与其它控制命令互斥；短命令用 0 tick 等待以免长时间阻塞 */
    SemaphoreHandle_t uart_mutex_{nullptr};

    QueueHandle_t print_job_queue_{nullptr};
    TaskHandle_t print_worker_task_{nullptr};
    SemaphoreHandle_t print_worker_exited_sem_{nullptr};
    /** 后台任务是否正在执行整段 UART 打印流程（含持锁阶段） */
    std::atomic<bool> background_print_active_{false};
    std::atomic<int> last_async_print_result_{ESP_OK};

    static void PrintWorkerEntry(void* opaque);

    bool TryLockUart(TickType_t wait_ticks);
    void UnlockUart();

    void DrainReceiveBuffer();
    esp_err_t WriteBytes(const uint8_t* data, size_t length);
    esp_err_t ReadSixByteFrameWithTimeout(uint8_t out_frame[6], uint32_t timeout_ms,
                                          uint32_t extend_after_header_ms = 0);
    esp_err_t ReadSixByteFrameSynced(uint8_t out_frame[6]);
    esp_err_t SendControlUnlocked(esp_serial_printer_protocol::ControlCommand command, int16_t data_field,
                                  int16_t* response_data);
    esp_err_t SendRowAndGetBufferCountUnlocked(const uint8_t row_mask[kRowBytes], int16_t* buffer_row_count);
    esp_err_t SetPrintDirectionUnlocked(uint8_t print_direction);
    esp_err_t PrintRasterRowsUnlocked(const uint8_t* row_data, size_t row_count, uint8_t print_direction,
                                      bool auto_feed);
    esp_err_t PrintBuiltInTestPatternUnlocked(uint8_t print_direction, bool auto_feed);

    /** 任务延时（毫秒），基于 FreeRTOS tick，不依赖 main/common/utility */
    void DelayMilliseconds(uint32_t milliseconds);
};

#endif  // _ESP_SERIAL_PRINTER_H_
