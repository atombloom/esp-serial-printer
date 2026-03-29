"""
串口图片打印核心流程（无 UI）：与固件交互的节奏控制、阶段 1/2 发送、等待完成、出纸。
默认参数与 GUI 版 ser_printfer2 保持一致。
"""

from __future__ import annotations

import time
from typing import Callable, Optional

import serial

from process_image import process_image_to_bytes
from proscess_ser import parse_response, send_row_data, send_cmd_and_parse_data

# ========== 与 GUI 版一致的默认配置 ==========
FIXED_WIDTH = 384
BYTE_PER_ROW = 48
DEFAULT_THRESHOLD = 178
DEFAULT_SERIAL_PORT = "COM36"
DEFAULT_BAUDRATE = 115200
# 默认图片路径（相对运行当前工作目录）
DEFAULT_IMAGE_PATH = "./py/234732.png"
# 打印方向：0 为上，1 为下（与现网 send_image_via_serial 一致）
DEFAULT_PRINT_DIRECTION = 1


class CmdCode:
    """固件串口命令码（与 ser_printfer2 一致）"""

    DEBUG = 0x00
    RESTART = 0x01
    START_PRINT = 0x02
    STOP_PRINT = 0x03
    MOTOR_CONTROL = 0x04
    MOTOR_STOP = 0x05
    READ_TEMP = 0x06
    READ_NO_PAPER = 0x07
    READ_ADC = 0x08
    READ_REMAIN_DATA = 0x09
    SET_PRINT_DIR = 0x0E


def run_serial_image_print(
    ser: serial.Serial,
    image_path: str,
    binary_threshold: int = DEFAULT_THRESHOLD,
    *,
    print_direction: int = DEFAULT_PRINT_DIRECTION,
    verbose_phase1_per_line: bool = True,
    log_print: Callable[[str], None] = print,
    on_phase2_row: Optional[Callable[[str, int, int], None]] = None,
) -> bool:
    """
    执行完整串口图片打印流程。

    :param ser: 已打开的 serial.Serial
    :param image_path: 图片路径
    :param binary_threshold: 二值化阈值
    :param print_direction: 打印方向 0 上 / 1 下
    :param verbose_phase1_per_line: 阶段 1 是否每行打印详情（False 时仅摘要）
    :param log_print: 日志输出（终端用 print，GUI 可包装）
    :param on_phase2_row: 每完成阶段 2 一行时回调 (行日志文案, 当前行号从 1 计, 总行数)
    :return: 是否完整成功结束
    """
    log = log_print

    log("\n=== 仅发送图片（按节奏控制）===")
    image_byte_rows = process_image_to_bytes(
        image_path=image_path, binary_threshold=binary_threshold, show_preview=False
    )
    if image_byte_rows is None:
        return False

    adaptive_height = len(image_byte_rows)
    log("\n--- 阶段1：预发送图像数据 ---")
    pre_send_stop = False
    pre_send_rows = 0

    for row_idx, row_data in enumerate(image_byte_rows):
        if verbose_phase1_per_line:
            log(f"\n发送第 {row_idx + 1}/{adaptive_height} 行")
        received_data = send_row_data(ser, row_data)

        if received_data is None:
            log(f"❌ 第{row_idx + 1}行发送失败，终止")
            return False

        parsed = parse_response(received_data)
        if parsed and len(parsed) == 3 and parsed[2] == 255:
            log("⚠️  对方缓冲区满，停止预发送")
            pre_send_stop = True
            pre_send_rows = row_idx + 1
            break

        pre_send_rows = row_idx + 1
        time.sleep(0.001)

    if not verbose_phase1_per_line:
        log(f"阶段1：结束，预先发送 {pre_send_rows} 行")
    if not pre_send_stop:
        log(f"⚠️  预发送完所有{pre_send_rows}行，缓冲区仍未满")

    # 设置打印方向（命令码 0x0E = 14）
    received_data = send_cmd_and_parse_data(ser, CmdCode.SET_PRINT_DIR, print_direction)
    if received_data is None:
        log("❌ 设置打印方向指令发送失败，终止")
        return False
    log(f"设置打印方向指令返回数据={received_data}")

    received_data = send_cmd_and_parse_data(ser, CmdCode.START_PRINT, 0)
    if received_data is None:
        log("❌ 触发打印指令发送失败，终止")
        return False
    if received_data == 0:
        log(f"❌ 触发打印指令返回数据错误，数据={received_data}，终止")
        return False
    log(f"触发打印指令返回数据={received_data}")

    time.sleep(0.01)

    log("\n--- 阶段2：续发送剩余图像数据 ---")
    last_row_index = pre_send_rows - 1

    for row_idx in range(pre_send_rows, adaptive_height):
        row_data = image_byte_rows[row_idx]
        received_data = send_row_data(ser, row_data)
        if received_data is None:
            log(f"❌ 第{row_idx + 1}行发送失败，终止")
            return False

        error_count = 0
        buff_row = -1

        for _retry in range(10):
            parsed = parse_response(received_data, False)
            if parsed is None:
                log(f"❌ 第{row_idx + 1}行应答解析失败，终止")
                return False
            _type_code, _cmd_code, data = parsed
            buff_row = data
            if data == 255:
                log("⚠️  对方缓冲区满，暂缓发送!!")
                error_count += 1
                if error_count >= 10:
                    log(f"❌ 第{row_idx + 1}行发送失败，终止")
                    return False
                time.sleep(0.01)
            else:
                break
            received_data = send_row_data(ser, row_data)
            if received_data is None:
                log(f"❌ 第{row_idx + 1}行发送失败，终止")
                return False

        if buff_row < 7:
            received_temp = -1
        else:
            received_temp = send_cmd_and_parse_data(ser, CmdCode.READ_TEMP, 0)
            if received_temp is None:
                log("❌ 检查温度指令发送失败，终止")
                return False
            if received_temp != -1:
                if received_temp < 200 or received_temp == 0xFFFF:
                    log(f"❌ 检查温度指令返回数据错误，数据={received_temp}，终止")
                    return False

        line_msg = (
            f"发送第 {row_idx + 1}/{adaptive_height} 行,buff={buff_row},"
            f"当前温度={received_temp}℃"
        )
        log(line_msg)
        if on_phase2_row is not None:
            on_phase2_row(line_msg, row_idx + 1, adaptive_height)

        last_row_index = row_idx

    sent_total = last_row_index + 1
    log(f"\n📤 图片发送完成！累计发送 {sent_total} 行")
    log("✅ 打印完成！")

    while True:
        data = send_cmd_and_parse_data(ser, CmdCode.READ_REMAIN_DATA, 0)
        if data is None:
            log("❌ 获取已保存数据量指令发送失败，终止")
            return False
        if data == 0:
            break
        time.sleep(0.1)
        log(f"剩余行数数据量={data}")

    if print_direction == 0:
        out_result = send_cmd_and_parse_data(ser, CmdCode.MOTOR_CONTROL, -300)
    else:
        out_result = send_cmd_and_parse_data(ser, CmdCode.MOTOR_CONTROL, 300)
    log(f"出纸指令返回数据={out_result}")
    return True
