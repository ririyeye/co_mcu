set_project(co_gd32)
set_arch("arm")
add_rules("mode.release", "mode.debug", "mode.releasedbg", "mode.minsizerel")
set_symbols("debug")
set_strip("all")
set_policy("build.optimization.lto", true)
add_rules("plugin.compile_commands.autoupdate")
set_policy("package.cmake_generator.ninja", true)
-- set_policy("diagnosis.check_build_deps", true)

includes("co_wq")
includes("toolchain")
includes("rule")
includes("gd32e11")

set_warnings("all", "extra", "pedantic")
set_languages("c17", "c++23")


