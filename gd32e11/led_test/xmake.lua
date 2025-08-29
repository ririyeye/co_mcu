target("led")
    set_kind("binary")

    -- add_deps("co_wq")
    add_deps("gd32e11")


    add_files(
        "led.cpp"
        -- , {optimize = "none"}
    )
    add_ldflags(
        "-Wl,-Map=build/led.map",
        "-Wl,--print-memory-usage",
        { force = true }
    )

    add_files(
        "../bsp/Firmware/gd32e11_script/gd32e11x_flash.lld", {
            rule = "modifyld",
            cfg = path.join(os.scriptdir(), "cfg.json"),
            origin = "led",
        }
    )


target_end()
