
includes("bsp")
includes("led_test")

target("gd32e11")
    set_kind("static")

    add_deps("co_wq")
    add_deps("gd32e11_bsp")

    add_includedirs(
        ".", {public=true}
    )

    add_files(
        "systick.cpp",
        "lock_imp.cpp"
    )

target_end()
