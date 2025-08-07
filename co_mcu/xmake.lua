target("co_mcu")

    add_linkgroups("co_mcu", { whole = true, public = true })

    set_kind("static")

    add_includedirs(
        "co_base/", {public=true}
    )
    add_includedirs(
        "workqueue/", {public=true}
    )
    add_includedirs(
        "./", {public=true}
    )
    add_files(
        "workqueue/workqueue.c"
        -- , {optimize = "none"}
    )
target_end()

