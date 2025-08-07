
includes("Firmware")


target("gd32e11_bsp")

    add_deps("co_mcu")

    add_linkgroups("gd32e11_bsp", { whole = true, public = true })

    set_kind("static")

    add_deps("gd32e11_bsp_firmware")

    add_files("syswork.cpp")

    add_includedirs(
        ".", {public=true}
    )

    add_files("co_driver/*.cpp")
    add_includedirs(
        "co_driver", {public=true}
    )

target_end()
