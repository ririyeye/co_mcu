
-- 选项：是否编译 gd32 相关库与示例（默认关闭）
option("gd32")
    set_default(false)
    set_showmenu(true)
    set_description("Enable building GD32 firmware libraries")
option_end()

-- 按需包含 gd32 工程
if has_config("gd32") then
    includes("gd32e11")
end

