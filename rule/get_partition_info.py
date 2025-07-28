import json
import re
import argparse
from typing import Tuple

import sys

sys.path.append(".")
from common_utils import ValueConverter


def decompose_value(value: int) -> Tuple[int, str]:
    """分解数值到最大单位表示（逆序单位）"""
    return ValueConverter.decompose_value(value, reverse=True)


def value_to_expr(value: int, raw: bool) -> str:
    """将数值转换为字符串表达式"""
    if raw:
        return str(value)
    val, unit = decompose_value(value)
    return f"{val}{unit}" if unit else str(val)


def convert_value(s: str) -> int:
    """数值转换函数"""
    return ValueConverter.convert_value(s)


def find_partition(config: dict, part_name: str) -> dict:
    """查找指定分区配置"""
    for idx, origin in enumerate(config["flash"]["origin"]):
        if origin["name"] == part_name:
            return origin
    raise ValueError(f"Partition '{part_name}' not found")


def calculate_partition_info(config: dict, part_name: str, raw: bool = False) -> dict:
    """计算分区的offset和length"""
    flash = config["flash"]
    total_length = convert_value(flash["length"])
    origins = flash["origin"]

    current_offset = 0
    target_part = None

    for origin in origins:
        # 计算offset
        if origin.get("offset", "-") == "-":
            offset_value = current_offset
        else:
            offset_value = convert_value(origin["offset"])

        # 计算length
        if origin["length"] == "-":
            if origin == origins[-1]:
                length_value = total_length - offset_value
            else:
                raise ValueError("Auto length only allowed for last partition")
        else:
            length_value = convert_value(origin["length"])

        # 记录目标分区
        if origin["name"] == part_name:
            target_part = {
                "name": part_name,
                "offset": offset_value,
                "length": length_value,
            }

        current_offset = offset_value + length_value

    if not target_part:
        raise ValueError(f"Partition {part_name} not found")

    # 计算FLASH地址
    flash_addr = convert_value(flash["addr"])
    origin_value = flash_addr + target_part["offset"]

    # 处理FLASH地址格式（继承gen_flash_origin.py的逻辑）
    flash_addr_str = flash["addr"].strip()
    origin_hex = f"0x{origin_value:08X}"

    # 处理RAM信息
    ram = config["ram"]
    ram_addr = convert_value(ram["addr"])
    ram_length = convert_value(ram["length"])

    return {
        "flash_origin": origin_hex,
        "flash_length": value_to_expr(target_part["length"], raw),
        "ram_origin": f"0x{ram_addr:08X}",
        "ram_length": value_to_expr(ram_length, raw),
    }


def process_ldscript(template_path: str, output_path: str, info: dict):
    """处理链接器脚本模板"""
    with open(template_path, "r", encoding="utf-8") as f:
        content = f.read()

    # 执行替换操作
    replacements = {
        "FLASH_ORIGIN": info["flash_origin"],
        "FLASH_LENGTH": info["flash_length"],
        "RAM_ORIGIN": info["ram_origin"],
        "RAM_LENGTH": info["ram_length"],
    }

    for key, value in replacements.items():
        content = re.sub(rf"\b{key}\b", value, content)

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(content)


def main():
    parser = argparse.ArgumentParser(description="Get flash partition info")
    parser.add_argument(
        "-f",
        "--function",
        default="boot",
        help="Partition name to query (e.g. boot)",
    )
    parser.add_argument(
        "-i",
        "--input",
        default="src_com/cfg.json",
        help="Input JSON config file path",
    )
    parser.add_argument(
        "-l",
        "--ldscript",
        default="gd32e11x_flash.lld",
        help="Linker script template path",
    )
    parser.add_argument("-o", "--output", help="Output file path")
    parser.add_argument("-r", "--raw", action="store_true", help="Output raw values without unit conversion")

    args = parser.parse_args()

    with open(args.input, "r", encoding="utf-8") as f:
        config = json.load(f)

    try:
        info = calculate_partition_info(config, args.function, args.raw)

        if args.output:  # 处理链接器脚本
            process_ldscript(args.ldscript, args.output, info)
        else:  # 保持原有控制台输出
            print(f"FLASH_ORIGIN={info['flash_origin']}")
            print(f"FLASH_LENGTH={info['flash_length']}")
            print(f"RAM_ORIGIN={info['ram_origin']}")
            print(f"RAM_LENGTH={info['ram_length']}")

    except Exception as e:
        print(f"Error: {str(e)}")
        exit(1)


if __name__ == "__main__":
    main()
