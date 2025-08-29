
toolchain("m4-arm-none-eabi")
    set_kind("standalone")


    if is_host("windows") then
        set_sdkdir("E:\\work\\xpack-arm-none-eabi-gcc-14.2.1-1.1\\")
    else
        set_sdkdir("/opt/toolchain/xpack-arm-none-eabi-gcc-14.2.1-1.1")
    end
    add_links(
        "stdc++",
        "supc++"
    )

    local mcu = { "-mcpu=cortex-m4", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard", "-mthumb" }
    table.join2(mcu, { "-fdata-sections", "-ffunction-sections" })

    add_cxflags(
        mcu,
        { force = true }
    )

    add_cxxflags(
        "-fcoroutines",
        "-fno-rtti -fno-exceptions -fno-threadsafe-statics"
    )

    add_defines("USE_EXCEPTION=0")

    add_asflags(
        mcu,
        "-x assembler-with-cpp",
        { force = true }
    )

    add_ldflags(
        mcu,
        "--specs=nano.specs",
        "-Wl,--undefined=_exit -Wl,--defsym=_exit=0",
        "-Wl,--gc-sections",
        -- "-u _printf_float",
        { force = true }
    )
toolchain_end()
