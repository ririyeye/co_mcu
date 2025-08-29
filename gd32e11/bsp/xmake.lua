

target("gd32e11_bsp")

    add_deps("co_wq")

    add_linkgroups("gd32e11_bsp", { whole = true, public = true })

    set_kind("static")

    add_files("syswork.cpp")

    add_includedirs(
        ".", {public=true}
    )

    add_files("Firmware/gcc_startup/startup_gd32e11x.S")
    add_files("Firmware/syscalls.c")
    add_files("Firmware/system_gd32e11x.c")

    add_files(
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_adc.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_crc.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_ctc.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_dac.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_dbg.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_dma.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_exmc.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_exti.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_fmc.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_fwdgt.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_gpio.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_i2c.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_misc.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_pmu.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_rcu.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_rtc.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_spi.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_timer.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_usart.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_wwdgt.c",
        "Firmware/GD32E11x_standard_peripheral/Source/gd32e11x_bkp.c"
        -- , {optimize = "none"}
    )
    add_includedirs(
        "Firmware/GD32E11x_standard_peripheral/Include", {public=true}
    )

    add_includedirs(
        "Firmware/Include", {public=true}
    )

    add_includedirs(
        "Firmware/Core/Include/", {public=true}
    )

    add_defines("USE_STDPERIPH_DRIVER", "GD32E11X")

    -- usb
    add_defines("USE_USB_FS")
    add_files(
        "Firmware/GD32E11x_usbfs_library/device/class/cdc/Source/cdc_acm_core.c",
        "Firmware/GD32E11x_usbfs_library/device/core/Source/*.c",
        "Firmware/GD32E11x_usbfs_library/driver/Source/drv_usbd_int.c",
        "Firmware/GD32E11x_usbfs_library/driver/Source/drv_usb_dev.c",
        "Firmware/GD32E11x_usbfs_library/driver/Source/drv_usb_core.c"
    )
    add_includedirs(
        "Firmware/GD32E11x_usbfs_library/ustd/common",
        "Firmware/GD32E11x_usbfs_library/ustd/class/cdc",
        "Firmware/GD32E11x_usbfs_library/driver/Include",
        "Firmware/GD32E11x_usbfs_library/device/core/Include",
        "Firmware/GD32E11x_usbfs_library/device/class/cdc/Include"
        -- , {public=true}
    )

    -- com
    add_includedirs(
        "co_driver", {public=true}
    )

    -- uart
    add_files(
        "co_driver/uart/co_uart.cpp"
        -- , {optimize = "none"}
    )

    add_includedirs(
        "co_driver/uart", {public=true}
    )

    -- spi
    add_files(
        "co_driver/spi/co_spi.cpp"
        -- , {optimize = "none"}
    )

    add_includedirs(
        "co_driver/spi", {public=true}
    )

    -- usb
    add_files(
        "co_driver/usb_cdc/co_usb_cdc.cpp",
        "co_driver/usb_cdc/gd32e11x_hw.c",
        "co_driver/usb_cdc/gd32e11x_it.c"
        -- , {optimize = "none"}
    )

    add_includedirs(
        "co_driver/usb_cdc", {public=true}
    )

target_end()
