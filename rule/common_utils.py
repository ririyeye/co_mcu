import re
import traceback


class ValueConverter:
    UNIT_MAP = {"G": 1024**3, "M": 1024**2, "K": 1024}

    @classmethod
    def decompose_value(cls, value: int, reverse=False) -> tuple[int, str]:
        """分解数值到最大单位表示 例如131072 -> (128, 'K')"""
        if value == 0:
            return (0, "")

        units = list(cls.UNIT_MAP.items())
        if reverse:
            units = reversed(units)

        for unit, size in units:
            if value % size == 0:
                return (value // size, unit)
        return (value, "")

    @classmethod
    def value_tuple_to_str(cls, value_tuple: tuple[int, str]) -> str:
        """将decompose_value的返回值转换为字符串表达式"""
        value, unit = value_tuple
        if unit:
            return f"{value} * {cls.UNIT_MAP[unit]}"
        return f"{value}"

    @classmethod
    def convert_value(cls, s: str) -> int:
        """基础数值转换函数，只返回整数值"""
        try:
            s = str(s).strip().upper()
            if not s or s == "-":
                return 0

            # 处理十六进制格式
            if s.startswith("0X"):
                return int(s, 16)

            # 处理二进制格式
            if s.startswith("0B"):
                return int(s[2:], 2)

            # 处理十进制数值（可能包含单位）
            match = re.match(r"^([+-]?)(\d+)([GMK]?)$", s)
            if match:
                sign, number, unit = match.groups()
                value = int(number)
                if sign == "-":
                    value = -value
                return value * cls.UNIT_MAP.get(unit, 1)

            # 处理纯十进制数字
            return int(s)

        except Exception as e:
            raise ValueError(f"数值转换错误: {str(e)}") from e
