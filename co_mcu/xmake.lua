target("co_task")

    add_linkgroups("co_task", { whole = true, public = true })

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

