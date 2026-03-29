#!/usr/bin/env python3
"""
纯终端图片串口打印：无 Qt/Matplotlib UI，默认参数与 ser_printfer2 一致。

用法示例：
  uv run py/print_image_terminal.py
  uv run py/print_image_terminal.py --image ./py/foo.png --port COM3 --threshold 178
"""

from __future__ import annotations

import argparse
import sys

import serial

from serial_print_core import (
    DEFAULT_BAUDRATE,
    DEFAULT_IMAGE_PATH,
    DEFAULT_PRINT_DIRECTION,
    DEFAULT_SERIAL_PORT,
    DEFAULT_THRESHOLD,
    run_serial_image_print,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="终端串口图片打印（默认配置与 GUI 版一致）")
    parser.add_argument(
        "--image",
        "-i",
        default=DEFAULT_IMAGE_PATH,
        help=f"图片路径（默认: {DEFAULT_IMAGE_PATH}）",
    )
    parser.add_argument(
        "--port",
        "-p",
        default=DEFAULT_SERIAL_PORT,
        help=f"串口名（默认: {DEFAULT_SERIAL_PORT}）",
    )
    parser.add_argument(
        "--baud",
        "-b",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"波特率（默认: {DEFAULT_BAUDRATE}）",
    )
    parser.add_argument(
        "--threshold",
        "-t",
        type=int,
        default=DEFAULT_THRESHOLD,
        help=f"二值化阈值 0~255（默认: {DEFAULT_THRESHOLD}）",
    )
    parser.add_argument(
        "--print-dir",
        type=int,
        choices=(0, 1),
        default=DEFAULT_PRINT_DIRECTION,
        help="打印方向：0 上，1 下（默认与固件现用一致：1）",
    )
    parser.add_argument(
        "--quiet-phase1",
        action="store_true",
        help="阶段1不逐行打印，仅摘要（减少终端输出）",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
            write_timeout=1,
        )
    except serial.SerialException as exc:
        print(f"❌ 打开串口失败: {exc}")
        return 1

    print(f"✅ 串口已打开: {args.port} @ {args.baud}")
    try:
        ok = run_serial_image_print(
            ser,
            args.image,
            args.threshold,
            print_direction=args.print_dir,
            verbose_phase1_per_line=not args.quiet_phase1,
        )
    finally:
        ser.close()
        print("🔌 串口已关闭")

    return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⚠️  用户中断")
        sys.exit(130)
