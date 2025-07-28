import os
import argparse
import json
from typing import Dict
from common_utils import ValueConverter


def load_config(config_path: str) -> Dict:
    """加载JSON配置文件"""
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


def calculate_partitions(config: Dict, bin_dir: str) -> Dict[str, Dict]:
    """计算全部分区信息并验证
    :param bin_dir: 二进制文件存放目录
    """
    flash = config["flash"]
    total_size = ValueConverter.convert_value(flash["length"])
    partitions = {}
    current_offset = 0
    ranges = []

    # 规范化路径并确保以斜杠结尾
    bin_dir = os.path.normpath(bin_dir) + os.sep

    for idx, part in enumerate(flash["origin"]):
        # 计算offset
        if part.get("offset", "-") == "-":
            offset = current_offset
        else:
            offset = ValueConverter.convert_value(part["offset"])
            # 如果显式指定offset，更新current_offset
            current_offset = offset

        # 计算length
        if part["length"] == "-":
            if part != flash["origin"][-1]:
                raise ValueError("非最后分区不能自动计算长度")
            length = total_size - offset
        else:
            length = ValueConverter.convert_value(part["length"])

        # 记录分区范围用于碰撞检测
        end = offset + length
        ranges.append((offset, end))

        # 检查分区是否超出flash范围
        if end > total_size:
            raise ValueError(
                f"分区 {part['name']} 结束地址 0x{end:X} 超过FLASH总大小 0x{total_size:X}"
            )

        partitions[part["name"]] = {
            "offset": offset,
            "length": length,
            "file": os.path.join(bin_dir, f"{part['name']}.bin"),
        }
        current_offset = end

    # 碰撞检测
    for i in range(len(ranges)):
        for j in range(i + 1, len(ranges)):
            a_start, a_end = ranges[i]
            b_start, b_end = ranges[j]
            if (a_start < b_end) and (b_start < a_end):
                raise ValueError(
                    f"分区冲突: {flash['origin'][i]['name']} (0x{a_start:X}-0x{a_end:X}) "
                    f"与 {flash['origin'][j]['name']} (0x{b_start:X}-0x{b_end:X}) 重叠"
                )

    # 处理最后分区4K对齐
    last_part = flash["origin"][-1]
    if last_part["length"] == "-":
        # 计算实际可用长度
        # 计算实际可用长度（4K对齐且不超过总大小）
        start = ranges[-1][0]
        max_possible = start + (total_size - start) // 4096 * 4096
        actual_length = max_possible - start

        # 更新最后分区长度和结束位置
        partitions[last_part["name"]]["length"] = actual_length
        ranges[-1] = (start, start + actual_length)
        print(
            f"最后分区 {last_part['name']} 已对齐到4K边界，实际长度: {actual_length}字节 ({actual_length//1024}K)"
        )

    return partitions


def validate_bin_file(file_path: str, max_size: int) -> bytes:
    """验证并加载二进制文件"""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Input file {file_path} not found")

    with open(file_path, "rb") as f:
        data = f.read()

    if len(data) > max_size:
        raise ValueError(
            f"File {file_path} size {len(data)} exceeds partition limit {max_size}"
        )

    # 填充到分区长度
    return data.ljust(max_size, b"\xff")


def generate_final_bin(
    config_path: str, output_path: str, bin_dir: str = "install/bin/"
):
    """生成最终二进制文件
    :param bin_dir: 二进制文件存放目录
    """
    config = load_config(config_path)
    partitions = calculate_partitions(config, bin_dir)
    flash_size = ValueConverter.convert_value(config["flash"]["length"])

    # 创建全FF的缓冲区
    final_bin = bytearray([0xFF] * flash_size)

    # 按分区写入数据
    for name, info in partitions.items():
        bin_data = validate_bin_file(info["file"], info["length"])
        offset = info["offset"]
        final_bin[offset : offset + len(bin_data)] = bin_data

    # 写入输出文件
    with open(output_path, "wb") as f:
        f.write(final_bin)
    print(f"Successfully generated {output_path} ({len(final_bin)//1024}K)")


def main():
    parser = argparse.ArgumentParser(description="Generate final binary image")
    parser.add_argument(
        "-c",
        "--config",
        default="cfg.json",
        help="Path to config file (default: cfg.json)",
    )
    parser.add_argument(
        "-o", "--output", required=True, help="Output file path"
    )
    parser.add_argument(
        "-b",
        "--bin-dir",
        default="install/bin",
        help="Binary files directory (default: install/bin)",
    )
    args = parser.parse_args()

    # 检查bin目录是否存在
    if not os.path.isdir(args.bin_dir):
        raise NotADirectoryError(
            f"Binary directory {args.bin_dir} does not exist"
        )

    try:
        generate_final_bin(args.config, args.output, args.bin_dir)
    except Exception as e:
        print(f"Error: {str(e)}")
        exit(1)


def validate_final_bin(config_path: str, bin_path: str):
    """校验最终二进制文件"""
    config = load_config(config_path)
    partitions = calculate_partitions(config, os.path.dirname(bin_path))
    flash_size = ValueConverter.convert_value(config["flash"]["length"])

    with open(bin_path, "rb") as f:
        data = f.read()

    if len(data) != flash_size:
        raise ValueError(
            f"文件大小不匹配: 预期 {flash_size} 字节，实际 {len(data)} 字节"
        )

    # 检查未使用区域是否为0xFF
    used_ranges = [
        (p["offset"], p["offset"] + p["length"]) for p in partitions.values()
    ]
    used_ranges.sort()

    # 检查间隙区域
    prev_end = 0
    for start, end in used_ranges:
        if start > prev_end:
            if any(b != 0xFF for b in data[prev_end:start]):
                raise ValueError(
                    f"未使用区域 0x{prev_end:X}-0x{start:X} 存在非0xFF数据"
                )
        prev_end = end

    # 检查最后分区对齐
    last_part = config["flash"]["origin"][-1]
    last_info = partitions[last_part["name"]]
    if (last_info["length"] % 4096) != 0:
        raise ValueError(
            f"最后分区 {last_part['name']} 未4K对齐，长度: {last_info['length']}"
        )

    print("校验通过，文件符合规范")


def main():
    parser = argparse.ArgumentParser(description="Generate final binary image")
    parser.add_argument(
        "-c",
        "--config",
        default="cfg.json",
        help="Path to config file (default: cfg.json)",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="install/firmware.bin",
        help="Output file path",
    )
    parser.add_argument(
        "-b",
        "--bin-dir",
        default="install/bin",
        help="Binary files directory (default: install/bin)",
    )
    parser.add_argument(
        "-v",
        "--validate",
        action="store_true",
        help="Validate existing output file instead of generating",
    )
    args = parser.parse_args()

    try:
        if args.validate:
            validate_final_bin(args.config, args.output)
        else:
            # 检查bin目录是否存在
            if not os.path.isdir(args.bin_dir):
                raise NotADirectoryError(
                    f"Binary directory {args.bin_dir} does not exist"
                )
            generate_final_bin(args.config, args.output, args.bin_dir)
    except Exception as e:
        print(f"Error: {str(e)}")
        exit(1)


if __name__ == "__main__":
    main()
