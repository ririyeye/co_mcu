
includes("bsp")
includes("led_test")

target("common")
    set_kind("static")

    add_deps("co_mcu")
    add_deps("gd32e11_bsp")

    add_includedirs(
        ".", {public=true}
    )

    add_files(
        "systick.cpp",
        "lock_imp.c"
    )

target_end()
