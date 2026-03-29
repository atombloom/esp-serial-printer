#ifndef _ESP_SERIAL_PRINTER_PROTOCOL_H_
#define _ESP_SERIAL_PRINTER_PROTOCOL_H_

#include <cstddef>
#include <cstdint>

/**
 * AtomBlool-Printer 串口协议（与 docs/串口上位机打印协议.md 一致）
 * 控制帧 6 字节；打印数据帧 51 字节；数据区 int16 为大端序。
 */

namespace esp_serial_printer_protocol {

constexpr uint8_t kFrameHeaderHigh = 0xAA;
constexpr uint8_t kFrameHeaderLow = 0xBB;
constexpr uint8_t kPacketTypeControl = 0x01;
constexpr uint8_t kPacketTypeRowData = 0x02;

constexpr size_t kControlFrameLength = 6;
constexpr size_t kRowPayloadBytes = 48;
constexpr size_t kRowFrameLength = 3 + kRowPayloadBytes;

/** 控制指令命令码（与协议表及 scripts/serial_print_core.py 中 CmdCode 对齐） */
enum class ControlCommand : uint8_t {
    kDebug = 0x00,
    kRestart = 0x01,
    kStartPrint = 0x02,
    kStopPrint = 0x03,
    kMotorControl = 0x04,
    kMotorStop = 0x05,
    kReadTemperature = 0x06,
    kReadNoPaper = 0x07,
    kReadAdc = 0x08,
    kReadRemainRows = 0x09,
    kSetHeatTime = 0x0A,
    kSetDotsPerFire = 0x0B,
    kReadHeatTime = 0x0C,
    kReadDotsPerFire = 0x0D,
    kSetPrintDirection = 0x0E,
    kReadPrintDirection = 0x0F,
};

/** 下位机应答中行缓冲满时返回的数据值（协议说明） */
constexpr int kBufferFullIndicator = 255;

inline void WriteInt16BigEndian(int16_t value, uint8_t* out_two_bytes) {
    out_two_bytes[0] = static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFF);
    out_two_bytes[1] = static_cast<uint8_t>(static_cast<uint16_t>(value) & 0xFF);
}

inline int16_t ReadInt16BigEndian(const uint8_t* two_bytes) {
    uint16_t u = (static_cast<uint16_t>(two_bytes[0]) << 8) | two_bytes[1];
    return static_cast<int16_t>(u);
}

/**
 * 组包控制指令：AA BB 01 | cmd | data(int16 BE)
 */
inline void BuildControlFrame(ControlCommand command, int16_t data_field, uint8_t out[kControlFrameLength]) {
    out[0] = kFrameHeaderHigh;
    out[1] = kFrameHeaderLow;
    out[2] = kPacketTypeControl;
    out[3] = static_cast<uint8_t>(command);
    WriteInt16BigEndian(data_field, &out[4]);
}

/**
 * 组包一行打印数据：AA BB 02 | 48 字节位图（每字节 8 点，共 384 点）
 */
inline void BuildRowDataFrame(const uint8_t row_mask[kRowPayloadBytes], uint8_t out[kRowFrameLength]) {
    out[0] = kFrameHeaderHigh;
    out[1] = kFrameHeaderLow;
    out[2] = kPacketTypeRowData;
    for (size_t index = 0; index < kRowPayloadBytes; ++index) {
        out[3 + index] = row_mask[index];
    }
}

/**
 * 解析 6 字节应答：校验帧头与类型码，输出最后两字节 int16（大端）
 * @return true 表示解析成功
 */
inline bool ParseControlStyleResponse(const uint8_t* frame_six_bytes, int16_t* out_data) {
    if (frame_six_bytes[0] != kFrameHeaderHigh || frame_six_bytes[1] != kFrameHeaderLow) {
        return false;
    }
    if (frame_six_bytes[2] != kPacketTypeControl) {
        return false;
    }
    if (out_data != nullptr) {
        *out_data = ReadInt16BigEndian(&frame_six_bytes[4]);
    }
    return true;
}

/** 打印数据行应答：仅校验帧头，数据区仍为最后 int16 BE（类型码可能为 0x02） */
inline bool ParseRowOrLooseResponse(const uint8_t* frame_six_bytes, int16_t* out_data) {
    if (frame_six_bytes[0] != kFrameHeaderHigh || frame_six_bytes[1] != kFrameHeaderLow) {
        return false;
    }
    if (out_data != nullptr) {
        *out_data = ReadInt16BigEndian(&frame_six_bytes[4]);
    }
    return true;
}

}  // namespace esp_serial_printer_protocol

#endif  // _ESP_SERIAL_PRINTER_PROTOCOL_H_
