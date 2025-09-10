set_project(co_mcu)
set_arch("arm")
add_rules("mode.release", "mode.debug", "mode.releasedbg", "mode.minsizerel")
set_symbols("debug")
set_strip("all")
set_policy("build.optimization.lto", true)
add_rules("plugin.compile_commands.autoupdate")
set_policy("package.cmake_generator.ninja", true)
-- set_policy("diagnosis.check_build_deps", true)

-- 基础组件始终包含
includes("co_wq")
includes("toolchain")
includes("rule")

-- 选项：是否编译 gd32 相关库与示例（默认关闭）
option("gd32")
	set_default(false)
	set_showmenu(true)
	set_description("Enable building GD32 firmware libraries")
option_end()

-- 选项：是否编译 gd32 led 测试示例（依赖 gd32）
option("gd32_test")
	add_deps("gd32")
	set_default(false)
	set_showmenu(true)
	set_description("Build GD32 LED test example")
option_end()

-- 按需包含 gd32 工程
if has_config("gd32") then
	includes("gd32e11")
end

set_warnings("all", "extra", "pedantic")
set_languages("c17", "c++23")


