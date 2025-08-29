
rule("modifyld")
    set_extensions(".lld")
    on_build_file(function(target, sourcefile, opt)
        -- imports
        import("core.base.option")
        import("core.theme.theme")
        import("core.project.config")
        import("core.project.depend")
        import("core.tool.compiler")
        import("utils.progress")

        local gen_cx = path.join(target:autogendir(), "rules", "ld", path.basename(sourcefile) .. ".ld")

        -- add linkfile
        target:add("ldflags", "-T" .. gen_cx, { force = true })
        target:data_add("linkdepfiles", gen_cx)

        local pypath = path.join(os.scriptdir(), "get_partition_info.py")
        local obj_args = { pypath, "-l", sourcefile, "-o", gen_cx }

        local fileconfig = target:fileconfig(sourcefile)
        local src_files = { sourcefile }
        local origin_name
        if fileconfig then
            if fileconfig.cfg then
                table.join2(obj_args, { "-i", fileconfig.cfg })
                table.join2(src_files, { fileconfig.cfg })
            end
            if fileconfig.origin then
                table.join2(obj_args, { "-f", fileconfig.origin })
                origin_name = fileconfig.origin
            else
                table.join2(obj_args, { "-f", target:name() })
                origin_name = target:name()
            end
        end

        local depargs = { obj_args }

        -- load dependent info
        local dependfile = target:dependfile(gen_cx)
        local dependinfo = option.get("rebuild") and {} or (depend.load(dependfile) or {})

        -- need build this object?
        if not depend.is_changed(dependinfo, { lastmtime = os.mtime(gen_cx), values = depargs }) then
            return
        end

        -- ensure the source file directory
        os.mkdir(path.directory(gen_cx))

        progress.show(opt.progress, "using origin %s fix link script:  %s -> %s", origin_name, sourcefile, gen_cx)
        os.vrunv("python", obj_args)

        -- update files and values to the dependent file
        dependinfo.values = depargs
        dependinfo.files = src_files
        depend.save(dependinfo, dependfile)
    end)
rule_end()

