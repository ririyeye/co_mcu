includes("@builtin/check")

target("bsp")

    add_linkgroups("bsp", {whole = true,public = true})

    set_kind("static")

    add_includedirs(
        "Firmware/cfg",
        "Firmware/CMSIS/Core/Include",
        "Firmware/CMSIS/GD/GD32E11x/Include",
        "Firmware/GD32E11x_standard_peripheral/Include/", {public=true}
    )

    add_files(
        "Firmware/bspinit/systick.c",
        "Firmware/bspinit/main.cpp"
        , {optimize = "none"}
    )

    if get_config("USING_CM_BACKTRACE") then
        add_includedirs("cm_backtrace")

        add_files(
            "cm_backtrace/fault_handler/gcc/cmb_fault.S",
            "cm_backtrace/cm_backtrace.c"
        )
    end

    -- stdperiph driver
    add_defines("USE_STDPERIPH_DRIVER", "GD32E11X")

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

    add_files(
        "Firmware/CMSIS/GD/GD32E11x/Source/system_gd32e11x.c"
        -- , {optimize = "none"}
    )
    add_includedirs(
        "Firmware/CMSIS/", {public=true}
    )

    -- usb
--     add_defines("USE_USB_FS")
--     add_files(
--         "Firmware/GD32E11x_usbfs_library/device/class/cdc/Source/cdc_acm_core.c",
--         "Firmware/GD32E11x_usbfs_library/device/core/Source/*.c",
--         "Firmware/GD32E11x_usbfs_library/driver/Source/drv_usbd_int.c",
--         "Firmware/GD32E11x_usbfs_library/driver/Source/drv_usb_dev.c",
--         "Firmware/GD32E11x_usbfs_library/driver/Source/drv_usb_core.c",
--         "Firmware/usb_ext/*.c"
--     )
--     add_includedirs(
--         "Firmware/GD32E11x_usbfs_library/ustd/common",
--         "Firmware/GD32E11x_usbfs_library/ustd/class/cdc",
--         "Firmware/GD32E11x_usbfs_library/driver/Include",
--         "Firmware/GD32E11x_usbfs_library/device/core/Include",
--         "Firmware/GD32E11x_usbfs_library/device/class/cdc/Include"
--     )
    -- common header
    add_includedirs(
        "list", {public=true}
    )

    -- init file
    add_files(
        "Firmware/gcc_startup/startup_gd32e11x.S"
    )

    -- ringbuf
    add_includedirs(
        "ringbuf", {public=true}
    )
    add_files(
        "ringbuf/ringbuffer.c",
        "ringbuf/ringbuffer_ext.c"
    )

    add_includedirs(
        "portable/GCC/ARM_CM4F", {public=true}
    )
    add_files(
        "portable/GCC/ARM_CM4F/port.c"
    )

    -- co_mcu
    add_includedirs(
        "co_mcu/", {public=true}
    )
    add_includedirs(
        "co_mcu/co_base/", {public=true}
    )
    add_files(
        "co_mcu/back_worker.cpp"
    )

target_end()

