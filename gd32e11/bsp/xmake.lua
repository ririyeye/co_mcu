

-- 外设功能开关：SPI / UART / USB CDC
-- 需求：这三个选项均默认开启，可单独关闭
option("gd32_spi")
    set_default(true)
    set_showmenu(true)
    set_description("Enable building GD32 SPI function")
option_end()

option("gd32_uart")
    set_default(true)
    set_showmenu(true)
    set_description("Enable building GD32 UART function")
option_end()

option("gd32_usb")
    set_default(true)
    set_showmenu(true)
    set_description("Enable building GD32 USB CDC function")
option_end()


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

    -- usb (受开关控制)
if has_config("gd32_usb") then
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
end

    -- com
    add_includedirs(
        "co_driver", {public=true}
    )

    -- uart (受开关控制)
if has_config("gd32_uart") then
    add_files(
        "co_driver/uart/co_uart.cpp"
        -- , {optimize = "none"}
    )

    add_includedirs(
        "co_driver/uart", {public=true}
    )
end

if has_config("gd32_spi") then
    -- spi
    add_files(
        "co_driver/spi/co_spi.cpp"
        -- , {optimize = "none"}
    )

    add_includedirs(
        "co_driver/spi", {public=true}
    )
end

if has_config("gd32_usb") then
    -- usb cdc driver layer
    add_files(
        "co_driver/usb_cdc/co_usb_cdc.cpp",
        "co_driver/usb_cdc/gd32e11x_hw.c",
        "co_driver/usb_cdc/gd32e11x_it.c"
        -- , {optimize = "none"}
    )

    add_includedirs(
        "co_driver/usb_cdc", {public=true}
    )
end

target_end()
