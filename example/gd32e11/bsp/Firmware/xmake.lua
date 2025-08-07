target("gd32e11_bsp_firmware")

    add_linkgroups("gd32e11_bsp_firmware", { whole = true, public = true })

    set_kind("static")

    add_files("gcc_startup/startup_gd32e11x.S")
    add_files("syscalls.c")
    add_files("system_gd32e11x.c")

    add_files(
        "GD32E11x_standard_peripheral/Source/gd32e11x_adc.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_crc.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_ctc.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_dac.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_dbg.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_dma.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_exmc.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_exti.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_fmc.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_fwdgt.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_gpio.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_i2c.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_misc.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_pmu.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_rcu.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_rtc.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_spi.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_timer.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_usart.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_wwdgt.c",
        "GD32E11x_standard_peripheral/Source/gd32e11x_bkp.c"
        -- , {optimize = "none"}
    )
    add_includedirs(
        "GD32E11x_standard_peripheral/Include", {public=true}
    )

    add_includedirs(
        "Include", {public=true}
    )

    add_includedirs(
        "Core/Include/", {public=true}
    )

    add_defines("USE_STDPERIPH_DRIVER", "GD32E11X")

    -- usb
    add_defines("USE_USB_FS")

target_end()
