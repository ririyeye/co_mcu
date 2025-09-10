
-- 始终包含底层 bsp（被 gd32 库目标使用）
includes("bsp")

-- 测试示例按需加入
if has_config("gd32_test") then
    includes("led_test")
end

target("gd32e11")
    set_kind("static")

    add_deps("gd32e11_bsp")

    add_includedirs(
        ".", {public=true}
    )

    add_files(
        "systick.cpp",
        "lock_imp.cpp"
    )

target_end()
