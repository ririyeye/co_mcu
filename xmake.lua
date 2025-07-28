set_project(h8040)
set_arch("arm")
add_rules("mode.release", "mode.debug", "mode.releasedbg", "mode.minsizerel")
set_symbols("debug")
set_strip("all")
set_policy("build.optimization.lto", true)
add_rules("plugin.compile_commands.autoupdate")
set_policy("package.cmake_generator.ninja", true)
-- set_policy("diagnosis.check_build_deps", true)

includes("toolchain")
includes("bsp")
includes("rule")

set_warnings("all", "extra", "pedantic")
set_languages("c23")

option("USING_CM_BACKTRACE")
    set_default(false)
option_end()

option("USING_SYS_SAVE")
    set_default(false)
option_end()

if get_config("USING_CM_BACKTRACE") then
    add_defines("ENABLE_CM_BACKTRACE=1")
else
    add_defines("ENABLE_CM_BACKTRACE=0")
end

target("boot")
    set_kind("binary")
    add_deps("bsp")

    add_ldflags(
        "-Wl,-Map=build/boot.map",
        -- "-Wl,--print-memory-usage",
        { force = true }
    )

    add_files(
        "bsp/Firmware/gd32e11_script/gd32e11x_flash.lld", {
            rule = "modifyld",
            cfg = "cfg.json",
        }
    )

    add_files(
        "src_app/main.c"
        -- , {optimize = "none"}
    )
    add_rules('gen_bin' , {cfg = 'cfg.json'})
    set_targetdir("build")
target_end()

