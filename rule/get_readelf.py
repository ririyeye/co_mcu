import re
import subprocess
from typing import Dict


def parse_memory_usage(text: str) -> Dict[str, int]:
    # 匹配LOAD段的关键字段（跳过Type和Align列）
    segment_pattern = r"^\s+LOAD\s+(0x[\da-fA-F]+)\s+(0x[\da-fA-F]+)\s+(0x[\da-fA-F]+)\s+(0x[\da-fA-F]+)\s+(0x[\da-fA-F]+)\s+(\w+)\s+(0x[\da-fA-F]+)"

    segments = re.findall(segment_pattern, text, re.MULTILINE)

    flash_size = 0
    ram_size = 0

    # 处理每个程序段
    for seg in segments:
        # 解包匹配到的6个组：offset, virt_addr, phys_addr, file_siz_hex, mem_siz_hex, flg, align
        (
            offset,
            virt_addr,
            phys_addr,
            file_siz_hex,
            mem_siz_hex,
            flags,
            align,
        ) = seg
        file_size = int(file_siz_hex, 16)
        mem_size = int(mem_siz_hex, 16)

        # 根据物理地址判断存储位置（FLASH地址空间为0x08000000开头）
        if phys_addr.startswith("0x08"):
            flash_size += file_size  # FLASH占用初始化数据大小
        if virt_addr.startswith("0x20"):
            ram_size += mem_size  # 纯RAM段（.bss、堆栈等）

    return {"FLASH": flash_size, "RAM": ram_size}


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Analyze ELF memory usage")
    parser.add_argument(
        "-i", "--input", help="Input text containing readelf -l output"
    )
    args = parser.parse_args()

    if args.input:
        input_text = args.input
    else:  # 默认硬件模式
        cmd = [
            "/opt/toolchain/xpack-arm-none-eabi-gcc-14.2.1-1.1/bin/arm-none-eabi-readelf",
            "-l",
            "/home/wangyang/fpv2_x_race/build/boot",
        ]
        result = subprocess.run(
            cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        input_text = result.stdout.decode("utf-8")

    # print(f"input: {input_text} B")
    memory_data = parse_memory_usage(input_text)

    print(f"FLASH={memory_data['FLASH']} B")
    print(f"RAM={memory_data['RAM']} B")
