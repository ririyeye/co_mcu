import json
import re
import traceback
import sys

sys.path.append(".")
from common_utils import ValueConverter


def generate_comment(s):
    """生成带安全过滤的注释"""
    if s is None:
        return "/*  */"
    # 过滤危险字符并截断长度
    safe_str = re.sub(r"[*/\\]", "", str(s))[:50]
    return f"/* {safe_str} */"


def generate_c_code(config, output_path):
    """生成C代码并处理异常"""
    try:
        flash = config["flash"]
        # 先获取flash总长度
        flash_length = ValueConverter.convert_value(flash["length"])
        flash_length_tmp = ValueConverter.decompose_value(flash_length)
        flash_length_expr = ValueConverter.value_tuple_to_str(flash_length_tmp)  # 总长度表达式
        flash_length_comment = generate_comment(flash["length"])  # 总长度注释
        
        current_offset_expr = "0"  # 初始化表达式偏移量
        current_offset_value = 0  # 初始化实际值偏移量
        origin_entries = []

        for idx, origin in enumerate(flash["origin"], 1):
            # 处理名称
            name = origin.get("name", "unnamed")[:32].replace('"', '\\"')
            if len(origin.get("name", "")) > 32:
                print(f"警告：分区名称'{origin['name']}'超过32字符，已截断")

            # 处理offset（支持"-"自动计算）
            if origin.get("offset", "") == "-":
                offset_value = current_offset_value
                offset_expr = current_offset_expr
            else:
                offset_value = ValueConverter.convert_value(
                    origin.get("offset", str(current_offset_value))
                )
                offset_tmp = ValueConverter.decompose_value(offset_value)
                offset_expr = ValueConverter.value_tuple_to_str(offset_tmp)

            # 处理特殊length值
            if origin["length"] == "-":
                if idx == len(flash["origin"]):  # 最后一个分区
                    # 使用原始单位表达式进行计算
                    length_value = flash_length - offset_value
                    length_tmp = ValueConverter.decompose_value(length_value)
                    length_expr = ValueConverter.value_tuple_to_str(length_tmp)
                else:
                    raise ValueError("非最后分区不能使用自动长度")
            else:
                length_value = ValueConverter.convert_value(origin["length"])
                length_tmp = ValueConverter.decompose_value(length_value)
                length_expr = ValueConverter.value_tuple_to_str(length_tmp)

            # 计算对齐空格（优化对齐逻辑）
            name_comment = generate_comment(f"分区{idx}")
            length_comment = generate_comment(origin["length"])
            offset_comment = generate_comment(origin.get("offset", "auto"))

            name_pad = max(4, 20 - (len(name) + 3))
            length_pad = max(4, 20 - len(length_expr))
            offset_pad = max(4, 20 - len(offset_expr))

            # 生成结构体条目
            entry = f"""    {{
        .name   = "{name}",{' ' * name_pad}{name_comment}
        .length = {length_expr},{' ' * length_pad}{length_comment}
        .offset = {offset_expr},{' ' * offset_pad}{offset_comment}
    }}"""
            origin_entries.append(entry)
            current_offset_value = offset_value + length_value
            current_tmp = ValueConverter.decompose_value(current_offset_value)
            current_offset_expr = ValueConverter.value_tuple_to_str(current_tmp)

        # 校验Flash总容量
        if current_offset_value > flash_length:
            print(
                f"错误：分区总长度超过Flash容量({current_offset_value} > {flash_length})"
            )
            return False

        # 处理Flash地址格式（使用预定义变量）
        addr = flash["addr"].strip()
        if addr.lower().startswith("0x"):
            hex_val = int(addr, 16)
            addr_expr = f"0x{hex_val:08X}"
        else:
            addr_value = ValueConverter.convert_value(addr)  # 复用已计算的addr_value
            addr_tmp = ValueConverter.decompose_value(addr_value)
            addr_expr = ValueConverter.value_tuple_to_str(addr_tmp)

        # 生成完整C代码
        c_code = f"""#include "flash_origin.h"

/* 自动生成文件 - 请勿手动修改 */
static const struct origin _origin[] = {{
{',\n'.join(origin_entries)}
}};

const struct flash _flash = {{
    .addr         = {addr_expr},        {generate_comment(flash['addr'])}
    .length       = {flash_length_expr},      {flash_length_comment}
    .p_origin     = _origin,
    .origin_count = sizeof(_origin) / sizeof(_origin[0]),
}};
"""

        with open(output_path, "w", encoding="utf-8") as f:
            f.write(c_code)
        return True

    except Exception as e:
        # print(f"代码生成失败: {str(e)}")
        print(traceback.format_exc())
        return False


if __name__ == "__main__":
    try:
        import argparse

        parser = argparse.ArgumentParser(
            description="Generate flash origin C code from JSON config"
        )
        parser.add_argument(
            "-i",
            "--input",
            default="src_com/cfg.json",
            help="Input JSON file path",
        )
        parser.add_argument(
            "-o",
            "--output",
            default="src_com/flash_origin_data_template.c",
            help="Output C file path",
        )
        args = parser.parse_args()

        with open(args.input, "r", encoding="utf-8") as f:
            config = json.load(f)

        if generate_c_code(config, args.output):
            print("origin gen ok!")
        else:
            print("生成过程中出现警告或错误，请检查输出！")

    except Exception as e:
        print(f"运行错误: {str(e)}")
        exit(1)
