#include "esp_serial_printer.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <cstdlib>
#include <cstring>
#include <cmath>

namespace {

constexpr const char* kLogTag = "EspSerialPrinter";

using esp_serial_printer_protocol::BuildControlFrame;
using esp_serial_printer_protocol::BuildRowDataFrame;
using esp_serial_printer_protocol::ControlCommand;
using esp_serial_printer_protocol::kBufferFullIndicator;
using esp_serial_printer_protocol::kControlFrameLength;
using esp_serial_printer_protocol::kFrameHeaderHigh;
using esp_serial_printer_protocol::kFrameHeaderLow;
using esp_serial_printer_protocol::kRowFrameLength;
using esp_serial_printer_protocol::ParseControlStyleResponse;
using esp_serial_printer_protocol::ParseRowOrLooseResponse;

/** 加大 RX 环形缓冲，减轻任务调度稍慢时应答字节被挤丢、表现为行超时 */
constexpr int kUartDriverRxBufferSize = 4096;
constexpr int kUartDriverTxBufferSize = 512;

/** 阶段 2 应答为 255 时重试等待的基础毫秒数（实际等待会随重试次数递增，见实现 cap） */
constexpr uint32_t kDefaultBufferFullRetryBaseMs = 14;
/** 255 重试：wait = buffer_full_retry_delay_ms * (1 + attempt)，不超过本值 */
constexpr uint32_t kBufferFullRetryBackoffCapMs = 900;
/** 行应答在已对齐 AA BB 后，为余下 4 字节至少保留的等待窗口（毫秒） */
constexpr uint32_t kDefaultRowFrameTailReserveMs = 4000;

EspSerialPrinterFlowOptions MakeDefaultFlowOptions() {
    EspSerialPrinterFlowOptions options = {};
    // 打印头行缓存约 50 行：再放慢并拉长行应答窗口，减少 12000ms 超时与 255 耗尽
    options.phase1_row_delay_ms = 10;
    options.phase2_row_delay_ms = 20; // 行间间隔
    options.phase2_high_print_head_buffer_rows = 18;
    options.phase2_high_print_head_extra_delay_ms = 60;
    options.buffer_full_retry_delay_ms = kDefaultBufferFullRetryBaseMs;
    options.buffer_full_max_attempts = 70;
    options.after_start_print_delay_ms = 120;
    options.remain_poll_interval_ms = 100;
    options.settle_after_last_row_ms = 0;
    options.response_read_timeout_ms = 2500;
    options.row_response_timeout_ms = 25000;
    options.row_read_timeout_extra_attempts = 1;
    // 超时后若重发，先多等一会再给下位机消化，减轻「双发」叠加 255
    options.row_read_timeout_retry_delay_ms = 280;
    options.row_frame_tail_reserve_ms = kDefaultRowFrameTailReserveMs;
    return options;
}

float RawTemperatureToCelsius(int16_t raw_value) {
    if (raw_value == static_cast<int16_t>(0xFFFF)) {
        return NAN;
    }
    // 协议为摄氏度×100；部分固件可能直接返回整型℃
    if (raw_value >= 1000 || raw_value <= -1000) {
        return static_cast<float>(raw_value) / 100.0f;
    }
    return static_cast<float>(raw_value);
}

/** 后台打印队列消息：kRasterOwnedBuffer 时 row_data 由 worker free */
struct BackgroundPrintJobMessage {
    enum class Kind : uint8_t {
        kStopWorker = 0,
        kRasterOwnedBuffer,
        kBuiltInTest,
    } kind;
    uint8_t* owned_row_data;
    size_t row_count;
    uint8_t print_direction;
    bool auto_feed;
};

/** 队列深度 2：执行中可再排队一条；析构时额外发送 kStopWorker */
constexpr UBaseType_t kPrintJobQueueDepth = 2;

}  // namespace

EspSerialPrinter::EspSerialPrinter(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin,
                                   uint32_t baud_rate)
    : uart_num_(uart_num),
      tx_pin_(tx_pin),
      rx_pin_(rx_pin),
      baud_rate_(baud_rate),
      initialized_(false),
      flow_(MakeDefaultFlowOptions()) {}

EspSerialPrinter::~EspSerialPrinter() {
    // 通知后台任务退出并等待其删除自身，避免与 UART 卸载竞态
    if (print_job_queue_ != nullptr && print_worker_task_ != nullptr) {
        BackgroundPrintJobMessage stop_message = {};
        stop_message.kind = BackgroundPrintJobMessage::Kind::kStopWorker;
        (void)xQueueSend(print_job_queue_, &stop_message, portMAX_DELAY);
        if (print_worker_exited_sem_ != nullptr) {
            (void)xSemaphoreTake(print_worker_exited_sem_, portMAX_DELAY);
        }
    }
    if (print_worker_exited_sem_ != nullptr) {
        vSemaphoreDelete(print_worker_exited_sem_);
        print_worker_exited_sem_ = nullptr;
    }
    if (print_job_queue_ != nullptr) {
        vQueueDelete(print_job_queue_);
        print_job_queue_ = nullptr;
    }
    print_worker_task_ = nullptr;

    if (uart_mutex_ != nullptr) {
        vSemaphoreDelete(uart_mutex_);
        uart_mutex_ = nullptr;
    }
    if (initialized_) {
        uart_driver_delete(uart_num_);
        initialized_ = false;
    }
}

EspSerialPrinterInitConfig EspSerialPrinter::DefaultInitConfig() {
    EspSerialPrinterInitConfig config = {};
    config.apply_print_direction = true;
    config.print_direction = kDefaultPrintDirection;
    config.send_stop_print_on_init = false;
    return config;
}

esp_err_t EspSerialPrinter::Init(const EspSerialPrinterInitConfig* init_config) {
    if (initialized_) {
        return ESP_OK;
    }
    if (tx_pin_ == GPIO_NUM_NC || rx_pin_ == GPIO_NUM_NC) {
        ESP_LOGE(kLogTag, "TX/RX pin must be set for printer UART");
        return ESP_ERR_INVALID_ARG;
    }

    uart_config_t uart_config = {
        .baud_rate = static_cast<int>(baud_rate_),
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err =
        uart_driver_install(uart_num_, kUartDriverRxBufferSize, kUartDriverTxBufferSize, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(kLogTag, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(uart_num_, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(kLogTag, "uart_param_config failed: %s", esp_err_to_name(err));
        uart_driver_delete(uart_num_);
        return err;
    }

    err = uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(kLogTag, "uart_set_pin failed: %s", esp_err_to_name(err));
        uart_driver_delete(uart_num_);
        return err;
    }

    initialized_ = true;

    EspSerialPrinterInitConfig printer_config = DefaultInitConfig();
    if (init_config != nullptr) {
        printer_config = *init_config;
    }

    if (printer_config.send_stop_print_on_init) {
        int16_t stop_ack = 0;
        err = SendControlUnlocked(ControlCommand::kStopPrint, 0, &stop_ack);
        if (err != ESP_OK) {
            ESP_LOGE(kLogTag, "init: stop print (0x03) failed: %s", esp_err_to_name(err));
            uart_driver_delete(uart_num_);
            initialized_ = false;
            return err;
        }
    }

    if (printer_config.apply_print_direction) {
        if (printer_config.print_direction > 1) {
            ESP_LOGE(kLogTag, "init: invalid print_direction %u", printer_config.print_direction);
            uart_driver_delete(uart_num_);
            initialized_ = false;
            return ESP_ERR_INVALID_ARG;
        }
        err = SetPrintDirectionUnlocked(printer_config.print_direction);
        if (err != ESP_OK) {
            ESP_LOGE(kLogTag, "init: set print direction failed: %s", esp_err_to_name(err));
            uart_driver_delete(uart_num_);
            initialized_ = false;
            return err;
        }
    }

    uart_mutex_ = xSemaphoreCreateMutex();
    if (uart_mutex_ == nullptr) {
        ESP_LOGE(kLogTag, "init: uart mutex create failed");
        uart_driver_delete(uart_num_);
        initialized_ = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(kLogTag, "UART %d init ok, TX=%d RX=%d baud=%lu", static_cast<int>(uart_num_),
             static_cast<int>(tx_pin_), static_cast<int>(rx_pin_), static_cast<unsigned long>(baud_rate_));
    if (printer_config.apply_print_direction) {
        ESP_LOGI(kLogTag, "init: applied print direction=%u (0x0E)", printer_config.print_direction);
    }
    return ESP_OK;
}

esp_err_t EspSerialPrinter::SetPrintDirection(uint8_t print_direction) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (print_direction > 1) {
        return ESP_ERR_INVALID_ARG;
    }
    // 打印或其它 UART 会话占用时立即失败，避免业务任务长时间阻塞
    if (!TryLockUart(0)) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t const err = SetPrintDirectionUnlocked(print_direction);
    UnlockUart();
    return err;
}

esp_err_t EspSerialPrinter::SetPrintDirectionUnlocked(uint8_t print_direction) {
    int16_t direction_ack = 0;
    return SendControlUnlocked(ControlCommand::kSetPrintDirection, static_cast<int16_t>(print_direction),
                               &direction_ack);
}

void EspSerialPrinter::SetFlowOptions(const EspSerialPrinterFlowOptions& options) {
    flow_ = options;
}

void EspSerialPrinter::DrainReceiveBuffer() {
    uint8_t discard_buffer[128];
    int received_length = 0;
    do {
        received_length = uart_read_bytes(uart_num_, discard_buffer, sizeof(discard_buffer), 0);
    } while (received_length > 0);
}

esp_err_t EspSerialPrinter::WriteBytes(const uint8_t* data, size_t length) {
    size_t written_total = 0;
    while (written_total < length) {
        int chunk = uart_write_bytes(uart_num_, reinterpret_cast<const char*>(data + written_total),
                                     length - written_total);
        if (chunk < 0) {
            return ESP_FAIL;
        }
        written_total += static_cast<size_t>(chunk);
    }
    return ESP_OK;
}

void EspSerialPrinter::DelayMilliseconds(uint32_t milliseconds) {
    if (milliseconds == 0) {
        return;
    }
    TickType_t delay_ticks = pdMS_TO_TICKS(milliseconds);
    if (delay_ticks < 1) {
        delay_ticks = 1;
    }
    vTaskDelay(delay_ticks);
}

esp_err_t EspSerialPrinter::ReadSixByteFrameWithTimeout(uint8_t out_frame[6], uint32_t timeout_ms,
                                                        uint32_t extend_after_header_ms) {
    uint8_t byte_previous = 0;
    const TickType_t start_ticks = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ticks < 1) {
        timeout_ticks = 1;
    }
    // 可延长：对齐 AA BB 后可为尾 4 字节追加窗口，避免总预算耗在同步上
    TickType_t deadline_ticks = start_ticks + timeout_ticks;

    while (xTaskGetTickCount() < deadline_ticks) {
        uint8_t byte_current = 0;
        const int read_count = uart_read_bytes(uart_num_, &byte_current, 1, pdMS_TO_TICKS(25));
        if (read_count != 1) {
            continue;
        }
        if (byte_previous == kFrameHeaderHigh && byte_current == kFrameHeaderLow) {
            out_frame[0] = kFrameHeaderHigh;
            out_frame[1] = kFrameHeaderLow;
            if (extend_after_header_ms > 0U) {
                TickType_t reserve_ticks = pdMS_TO_TICKS(extend_after_header_ms);
                if (reserve_ticks < 1) {
                    reserve_ticks = 1;
                }
                const TickType_t extended_deadline = xTaskGetTickCount() + reserve_ticks;
                if (extended_deadline > deadline_ticks) {
                    deadline_ticks = extended_deadline;
                }
            }
            size_t tail_received = 0;
            while (tail_received < 4) {
                if (xTaskGetTickCount() >= deadline_ticks) {
                    return ESP_ERR_TIMEOUT;
                }
                const int tail_read =
                    uart_read_bytes(uart_num_, out_frame + 2 + tail_received, 4 - tail_received, pdMS_TO_TICKS(100));
                if (tail_read < 0) {
                    return ESP_FAIL;
                }
                if (tail_read == 0) {
                    continue;
                }
                tail_received += static_cast<size_t>(tail_read);
            }
            return ESP_OK;
        }
        byte_previous = byte_current;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t EspSerialPrinter::ReadSixByteFrameSynced(uint8_t out_frame[6]) {
    return ReadSixByteFrameWithTimeout(out_frame, flow_.response_read_timeout_ms);
}

bool EspSerialPrinter::TryLockUart(TickType_t wait_ticks) {
    if (uart_mutex_ == nullptr) {
        return false;
    }
    return xSemaphoreTake(uart_mutex_, wait_ticks) == pdTRUE;
}

void EspSerialPrinter::UnlockUart() {
    if (uart_mutex_ != nullptr) {
        (void)xSemaphoreGive(uart_mutex_);
    }
}

esp_err_t EspSerialPrinter::SendControlUnlocked(esp_serial_printer_protocol::ControlCommand command,
                                                int16_t data_field, int16_t* response_data) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    DrainReceiveBuffer();

    uint8_t request_frame[kControlFrameLength];
    BuildControlFrame(command, data_field, request_frame);
    esp_err_t err = WriteBytes(request_frame, sizeof(request_frame));
    if (err != ESP_OK) {
        ESP_LOGE(kLogTag, "SendControl write failed");
        return err;
    }

    uint8_t response_frame[kControlFrameLength];
    err = ReadSixByteFrameSynced(response_frame);
    if (err != ESP_OK) {
        ESP_LOGE(kLogTag, "SendControl read timeout (cmd=0x%02X, timeout_ms=%u)",
                 static_cast<unsigned>(command), static_cast<unsigned>(flow_.response_read_timeout_ms));
        return err;
    }

    int16_t parsed_data = 0;
    if (!ParseControlStyleResponse(response_frame, &parsed_data)) {
        ESP_LOGE(kLogTag, "SendControl bad response frame");
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (response_data != nullptr) {
        *response_data = parsed_data;
    }
    return ESP_OK;
}

esp_err_t EspSerialPrinter::SendRowAndGetBufferCountUnlocked(const uint8_t row_mask[kRowBytes],
                                                             int16_t* buffer_row_count) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint32_t row_timeout_ms =
        (flow_.row_response_timeout_ms != 0U) ? flow_.row_response_timeout_ms : flow_.response_read_timeout_ms;
    const unsigned max_attempts = 1U + flow_.row_read_timeout_extra_attempts;

    uint8_t response_frame[kControlFrameLength];
    esp_err_t last_err = ESP_OK;

    for (unsigned attempt_index = 0; attempt_index < max_attempts; ++attempt_index) {
        // 与 Python send_row_data 一致：正常一发一收不先清空 RX；仅超时后重发前丢弃残留，避免帧错位
        if (attempt_index > 0U) {
            DrainReceiveBuffer();
        }

        uint8_t request_frame[kRowFrameLength];
        BuildRowDataFrame(row_mask, request_frame);
        esp_err_t err = WriteBytes(request_frame, sizeof(request_frame));
        if (err != ESP_OK) {
            return err;
        }

        err = ReadSixByteFrameWithTimeout(response_frame, row_timeout_ms, flow_.row_frame_tail_reserve_ms);
        last_err = err;
        if (err == ESP_OK) {
            int16_t parsed = 0;
            if (!ParseRowOrLooseResponse(response_frame, &parsed)) {
                return ESP_ERR_INVALID_RESPONSE;
            }
            if (buffer_row_count != nullptr) {
                *buffer_row_count = parsed;
            }
            return ESP_OK;
        }
        if (err != ESP_ERR_TIMEOUT) {
            return err;
        }
        if (attempt_index + 1U < max_attempts) {
            ESP_LOGW(kLogTag, "row response timeout (%u ms), retry %u/%u after %u ms", static_cast<unsigned>(row_timeout_ms),
                     static_cast<unsigned>(attempt_index + 2U), static_cast<unsigned>(max_attempts),
                     static_cast<unsigned>(flow_.row_read_timeout_retry_delay_ms));
            DelayMilliseconds(flow_.row_read_timeout_retry_delay_ms);
        }
    }

    return last_err;
}

esp_err_t EspSerialPrinter::ReadDeviceTemperature(int16_t* raw_out, float* celsius_out) {
    if (raw_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!TryLockUart(0)) {
        return ESP_ERR_INVALID_STATE;
    }
    int16_t raw_value = 0;
    esp_err_t err = SendControlUnlocked(ControlCommand::kReadTemperature, 0, &raw_value);
    if (err != ESP_OK) {
        UnlockUart();
        return err;
    }
    if (raw_value == static_cast<int16_t>(0xFFFF)) {
        UnlockUart();
        return ESP_ERR_INVALID_RESPONSE;
    }
    *raw_out = raw_value;
    if (celsius_out != nullptr) {
        *celsius_out = RawTemperatureToCelsius(raw_value);
    }
    UnlockUart();
    return ESP_OK;
}

esp_err_t EspSerialPrinter::FeedPaper(int16_t steps) {
    if (!TryLockUart(0)) {
        return ESP_ERR_INVALID_STATE;
    }
    int16_t ack = 0;
    esp_err_t err = SendControlUnlocked(ControlCommand::kMotorControl, steps, &ack);
    UnlockUart();
    if (err != ESP_OK) {
        return err;
    }
    (void)ack;
    return ESP_OK;
}

esp_err_t EspSerialPrinter::StopMotor() {
    if (!TryLockUart(0)) {
        return ESP_ERR_INVALID_STATE;
    }
    int16_t ack = 0;
    esp_err_t const err = SendControlUnlocked(ControlCommand::kMotorStop, 0, &ack);
    UnlockUart();
    return err;
}

esp_err_t EspSerialPrinter::PrintRasterRows(const uint8_t* row_data, size_t row_count,
                                            uint8_t print_direction, bool auto_feed) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (row_data == nullptr || row_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (print_direction > 1) {
        return ESP_ERR_INVALID_ARG;
    }
    // 同步打印：独占 UART 直至整图结束（内部 vTaskDelay 会让出 CPU，但仍不宜在关键路径上直接调用）
    if (!TryLockUart(portMAX_DELAY)) {
        return ESP_ERR_INVALID_STATE;
    }
    background_print_active_.store(true, std::memory_order_release);
    esp_err_t const err = PrintRasterRowsUnlocked(row_data, row_count, print_direction, auto_feed);
    background_print_active_.store(false, std::memory_order_release);
    UnlockUart();
    return err;
}

esp_err_t EspSerialPrinter::PrintFixedRasterImage(const uint8_t* raster_image, uint8_t print_direction,
                                                  bool auto_feed) {
    if (raster_image == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    // 与 PrintRasterRows 相同流程，行数固定为 kFixedRasterImageRows（384×48=18432）
    return PrintRasterRows(raster_image, kFixedRasterImageRows, print_direction, auto_feed);
}

esp_err_t EspSerialPrinter::PrintRasterRowsUnlocked(const uint8_t* row_data, size_t row_count,
                                                    uint8_t print_direction, bool auto_feed) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (row_data == nullptr || row_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (print_direction > 1) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t pre_send_rows = 0;
    bool pre_send_stopped_early = false;

    for (size_t row_index = 0; row_index < row_count; ++row_index) {
        int16_t phase1_response = 0;
        esp_err_t err = SendRowAndGetBufferCountUnlocked(row_data + (row_index * kRowBytes), &phase1_response);
        if (err != ESP_OK) {
            ESP_LOGE(kLogTag, "phase1 row %u failed", static_cast<unsigned>(row_index + 1));
            return err;
        }
        if (phase1_response == kBufferFullIndicator) {
            pre_send_stopped_early = true;
            pre_send_rows = row_index + 1;
            break;
        }
        pre_send_rows = row_index + 1;
        // 与阶段 2 相同：按行应答里的打印头占用节排，避免阶段 1 打满 ~50 行头缓存
        if (flow_.phase2_high_print_head_buffer_rows > 0 && phase1_response >= 0 &&
            static_cast<uint32_t>(phase1_response) >= flow_.phase2_high_print_head_buffer_rows) {
            DelayMilliseconds(flow_.phase2_high_print_head_extra_delay_ms);
        }
        DelayMilliseconds(flow_.phase1_row_delay_ms);
    }

    if (!pre_send_stopped_early) {
        ESP_LOGW(kLogTag, "phase1: sent all %u rows, buffer never reported full (255)",
                 static_cast<unsigned>(pre_send_rows));
    }

    esp_err_t err = SetPrintDirectionUnlocked(print_direction);
    if (err != ESP_OK) {
        ESP_LOGE(kLogTag, "set print direction failed");
        return err;
    }

    int16_t start_ack = 0;
    err = SendControlUnlocked(ControlCommand::kStartPrint, 0, &start_ack);
    if (err != ESP_OK) {
        ESP_LOGE(kLogTag, "start print failed");
        return err;
    }
    if (start_ack == 0) {
        ESP_LOGE(kLogTag, "start print rejected (protocol: buffer line count 0)");
        return ESP_ERR_INVALID_STATE;
    }

    DelayMilliseconds(flow_.after_start_print_delay_ms);
    // 阶段 2 开始前清 RX，避免阶段 1/控制帧残留半帧干扰首行应答同步
    DrainReceiveBuffer();

    size_t last_completed_row_index = (pre_send_rows > 0) ? (pre_send_rows - 1) : 0;

    for (size_t row_index = pre_send_rows; row_index < row_count; ++row_index) {
        int16_t buffer_row = 0;
        bool send_ok = false;
        for (unsigned attempt = 0; attempt < flow_.buffer_full_max_attempts; ++attempt) {
            err = SendRowAndGetBufferCountUnlocked(row_data + (row_index * kRowBytes), &buffer_row);
            if (err != ESP_OK) {
                ESP_LOGE(kLogTag, "phase2 row %u send failed", static_cast<unsigned>(row_index + 1));
                return err;
            }
            if (buffer_row != kBufferFullIndicator) {
                send_ok = true;
                break;
            }
            if (attempt + 1 >= flow_.buffer_full_max_attempts) {
                ESP_LOGE(kLogTag, "phase2 row %u buffer full too many times",
                         static_cast<unsigned>(row_index + 1));
                return ESP_ERR_TIMEOUT;
            }
            // 持续 255 时递增等待（上限 kBufferFullRetryBackoffCapMs），比固定 10ms 更易撑过打印头消化慢
            const uint32_t base_full_ms = flow_.buffer_full_retry_delay_ms;
            const uint32_t scaled_full_ms = base_full_ms * (1U + static_cast<uint32_t>(attempt));
            const uint32_t wait_full_ms =
                (scaled_full_ms < kBufferFullRetryBackoffCapMs) ? scaled_full_ms : kBufferFullRetryBackoffCapMs;
            DelayMilliseconds(wait_full_ms);
        }
        if (!send_ok) {
            return ESP_ERR_TIMEOUT;
        }

        // 行应答中的 buffer_row 为下位机打印头侧行缓存占用；约 50 行容量时勿再插 0x06 读温度等控制，
        // 仅用可配置延时节排，避免下位机忙于走纸时控制帧应答超时。
        if (flow_.phase2_high_print_head_buffer_rows > 0 &&
            buffer_row >= 0 && static_cast<uint32_t>(buffer_row) >= flow_.phase2_high_print_head_buffer_rows) {
            DelayMilliseconds(flow_.phase2_high_print_head_extra_delay_ms);
        }

        if (flow_.phase2_row_delay_ms > 0) {
            DelayMilliseconds(flow_.phase2_row_delay_ms);
        }

        last_completed_row_index = row_index;
    }

    ESP_LOGI(kLogTag, "raster send done, last row index=%u",
             static_cast<unsigned>(last_completed_row_index));

    if (flow_.settle_after_last_row_ms > 0) {
        DelayMilliseconds(flow_.settle_after_last_row_ms);
    }

    while (true) {
        int16_t remain_raw = 0;
        err = SendControlUnlocked(ControlCommand::kReadRemainRows, 0, &remain_raw);
        if (err != ESP_OK) {
            return err;
        }
        const uint16_t remain_rows = static_cast<uint16_t>(remain_raw);
        if (remain_rows == 0) {
            break;
        }
        ESP_LOGD(kLogTag, "remain rows in buffer: %u", static_cast<unsigned>(remain_rows));
        DelayMilliseconds(flow_.remain_poll_interval_ms);
    }

    if (auto_feed) {
        const int16_t feed_steps = (print_direction == 0) ? -300 : 300;
        int16_t motor_ack = 0;
        err = SendControlUnlocked(ControlCommand::kMotorControl, feed_steps, &motor_ack);
        if (err != ESP_OK) {
            return err;
        }
        ESP_LOGI(kLogTag, "auto feed steps=%d, ack=%d", static_cast<int>(feed_steps),
                 static_cast<int>(motor_ack));
    }

    return ESP_OK;
}

esp_err_t EspSerialPrinter::PrintBuiltInTestPatternUnlocked(uint8_t print_direction, bool auto_feed) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (print_direction > 1) {
        return ESP_ERR_INVALID_ARG;
    }
    // 与 EnqueuePrintBuiltInTestPatternAsync 配套：不依赖外部头文件，首次调用时填充固定尺寸缓冲（约 18KiB BSS）
    static uint8_t s_built_in_raster[kFixedRasterImageBytes];
    static bool s_built_in_raster_ready = false;
    if (!s_built_in_raster_ready) {
        for (size_t row_index = 0; row_index < kFixedRasterImageRows; ++row_index) {
            uint8_t* const row_ptr = s_built_in_raster + row_index * kRowBytes;
            // 横向 16 行为一组黑白带，便于肉眼看走纸与丢行
            const uint8_t band_fill = ((row_index / 16) & 1U) ? static_cast<uint8_t>(0xFF) : static_cast<uint8_t>(0x00);
            std::memset(row_ptr, band_fill, kRowBytes);
        }
        s_built_in_raster_ready = true;
    }
    return PrintRasterRowsUnlocked(s_built_in_raster, kFixedRasterImageRows, print_direction, auto_feed);
}

void EspSerialPrinter::PrintWorkerEntry(void* opaque) {
    auto* printer = static_cast<EspSerialPrinter*>(opaque);
    BackgroundPrintJobMessage message;
    for (;;) {
        if (xQueueReceive(printer->print_job_queue_, &message, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (message.kind == BackgroundPrintJobMessage::Kind::kStopWorker) {
            break;
        }

        if (!printer->TryLockUart(portMAX_DELAY)) {
            printer->last_async_print_result_.store(static_cast<int>(ESP_ERR_INVALID_STATE), std::memory_order_relaxed);
            if (message.owned_row_data != nullptr) {
                std::free(message.owned_row_data);
            }
            continue;
        }
        printer->background_print_active_.store(true, std::memory_order_release);
        esp_err_t job_error = ESP_OK;
        if (message.kind == BackgroundPrintJobMessage::Kind::kBuiltInTest) {
            job_error = printer->PrintBuiltInTestPatternUnlocked(message.print_direction, message.auto_feed);
        } else {
            job_error = printer->PrintRasterRowsUnlocked(message.owned_row_data, message.row_count, message.print_direction,
                                                         message.auto_feed);
        }
        if (message.owned_row_data != nullptr) {
            std::free(message.owned_row_data);
        }
        printer->last_async_print_result_.store(static_cast<int>(job_error), std::memory_order_relaxed);
        printer->UnlockUart();
        printer->background_print_active_.store(false, std::memory_order_release);
    }
    (void)xSemaphoreGive(printer->print_worker_exited_sem_);
    vTaskDelete(nullptr);
}

esp_err_t EspSerialPrinter::StartPrintWorker(uint32_t stack_words, UBaseType_t priority) {
    if (!initialized_ || uart_mutex_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (print_worker_task_ != nullptr) {
        return ESP_OK;
    }
    print_worker_exited_sem_ = xSemaphoreCreateBinary();
    if (print_worker_exited_sem_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    print_job_queue_ = xQueueCreate(kPrintJobQueueDepth, sizeof(BackgroundPrintJobMessage));
    if (print_job_queue_ == nullptr) {
        vSemaphoreDelete(print_worker_exited_sem_);
        print_worker_exited_sem_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    const BaseType_t created = xTaskCreate(PrintWorkerEntry, "esp_serial_printer", static_cast<uint32_t>(stack_words),
                                           this, priority, &print_worker_task_);
    if (created != pdPASS) {
        vQueueDelete(print_job_queue_);
        print_job_queue_ = nullptr;
        vSemaphoreDelete(print_worker_exited_sem_);
        print_worker_exited_sem_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t EspSerialPrinter::EnqueuePrintRasterRowsAsync(const uint8_t* row_data, size_t row_count,
                                                        uint8_t print_direction, bool auto_feed) {
    if (print_worker_task_ == nullptr || print_job_queue_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (row_data == nullptr || row_count == 0 || print_direction > 1) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t byte_count = row_count * kRowBytes;
    uint8_t* owned_copy = static_cast<uint8_t*>(std::malloc(byte_count));
    if (owned_copy == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    std::memcpy(owned_copy, row_data, byte_count);
    BackgroundPrintJobMessage job_message = {};
    job_message.kind = BackgroundPrintJobMessage::Kind::kRasterOwnedBuffer;
    job_message.owned_row_data = owned_copy;
    job_message.row_count = row_count;
    job_message.print_direction = print_direction;
    job_message.auto_feed = auto_feed;
    if (xQueueSend(print_job_queue_, &job_message, 0) != pdTRUE) {
        std::free(owned_copy);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t EspSerialPrinter::EnqueuePrintFixedRasterImageAsync(const uint8_t* raster_image,
                                                              uint8_t print_direction, bool auto_feed) {
    if (raster_image == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return EnqueuePrintRasterRowsAsync(raster_image, kFixedRasterImageRows, print_direction, auto_feed);
}

esp_err_t EspSerialPrinter::EnqueuePrintBuiltInTestPatternAsync(uint8_t print_direction, bool auto_feed) {
    if (print_worker_task_ == nullptr || print_job_queue_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (print_direction > 1) {
        return ESP_ERR_INVALID_ARG;
    }
    BackgroundPrintJobMessage job_message = {};
    job_message.kind = BackgroundPrintJobMessage::Kind::kBuiltInTest;
    job_message.owned_row_data = nullptr;
    job_message.row_count = 0;
    job_message.print_direction = print_direction;
    job_message.auto_feed = auto_feed;
    if (xQueueSend(print_job_queue_, &job_message, 0) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

EspSerialPrinterJobState EspSerialPrinter::GetPrintJobState() const {
    if (background_print_active_.load(std::memory_order_acquire)) {
        return EspSerialPrinterJobState::kBusy;
    }
    if (print_job_queue_ != nullptr && uxQueueMessagesWaiting(print_job_queue_) > 0U) {
        return EspSerialPrinterJobState::kBusy;
    }
    return EspSerialPrinterJobState::kIdle;
}

esp_err_t EspSerialPrinter::GetLastAsyncPrintResult() const {
    return static_cast<esp_err_t>(last_async_print_result_.load(std::memory_order_relaxed));
}
