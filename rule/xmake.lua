rule("gen_origin")
    set_extensions(".json")
    on_load(function(target)
        target:add("includedirs", path.join(os.scriptdir(), "origin"), { public = true })
        local origin_file = path.join(os.scriptdir(), "origin/flash_origin_fun.c")
        local origin_obj = target:objectfile(origin_file)
        target:add("files", origin_file)
    end)

    before_build_file(function(target, sourcefile, opt)
        -- imports
        import("core.base.option")
        import("core.theme.theme")
        import("core.project.config")
        import("core.project.depend")
        import("core.tool.compiler")
        import("utils.progress")

        local gen_cx = path.join(target:autogendir(), "rules", "flash", path.basename(sourcefile) .. ".c")
        local objectfile = target:objectfile(gen_cx)
        table.insert(target:objectfiles(), objectfile)

        local pypath = path.join(os.scriptdir(), "gen_flash_origin.py")
        local obj_args = { pypath, "-i", sourcefile, "-o", gen_cx }

        local depargs = { obj_args }

        -- load dependent info
        local dependfile = target:dependfile(objectfile)
        local dependinfo = option.get("rebuild") and {} or (depend.load(dependfile) or {})

        -- need build this object?
        if not depend.is_changed(dependinfo, { lastmtime = os.mtime(objectfile), values = depargs }) then
            return
        end

        -- ensure the source file directory
        os.mkdir(path.directory(gen_cx))

        progress.show(opt.progress, "gen flash origin info:  %s -> %s", sourcefile, gen_cx)
        os.vrunv("python", obj_args)
        progress.show(opt.progress, "compiling.$(mode) %s -> %s", gen_cx, objectfile)
        compiler.compile(gen_cx, objectfile, { target = target })

        -- show build progress
        -- update files and values to the dependent file
        dependinfo.values = depargs
        dependinfo.files = { sourcefile }
        depend.save(dependinfo, dependfile)
    end)
rule_end()

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

rule("gen_bin")
    after_install(function(target)
        import("utils.progress")

        local installpy = target:installdir()
        local elf = target:targetfile()
        local binary = path.join(installpy, "bin", target:name() .. ".bin")

        -- add objectfile
        local gcc = target:tool("cc")
        local objpath, _ = path.filename(gcc):gsub("gcc", "objcopy")
        local objcopy = path.join(path.directory(gcc), objpath)

        local sizepath, _ = path.filename(gcc):gsub("gcc", "readelf")
        local size = path.join(path.directory(gcc), sizepath)

        -- add commands
        os.vrunv(objcopy, { "-O", "binary", elf, binary })
        -- show build progress
        print("objcopy %s -> %s", elf, binary)

        -- add commands
        local outdata, errdata, exitcode = os.iorunv(size, { "-l", elf })

        if not exitcode then
            local py_elf = path.join(os.scriptdir(), "get_readelf.py")
            local outdata2, errdata, exitcode = os.iorunv("python", { py_elf, "-i", outdata })
            -- outdata2
            -- FLASH=30952 B
            -- RAM=32316 B
            -- 解析size输出
            local flash_txt = outdata2:match("FLASH=(%d+)")
            local ram_txt = outdata2:match("RAM=(%d+)")
            local total_flash_real = tonumber(flash_txt) or tonumber(0)
            local total_ram_real = tonumber(ram_txt) or tonumber(0)

            local total_flash_len = 0
            local total_ram_len = 0

            local cfgfile = target:extraconf("rules", "gen_bin", "cfg")
            if cfgfile then
                local pypath = path.join(os.scriptdir(), "get_partition_info.py")
                local obj_args = { pypath, "-i", cfgfile, "-r", "-f", target:name(), }
                local outdata, errdata, exitcode = os.iorunv("python", obj_args)
                if not exitcode then
                    -- Parse flash and ram lengths from Python output
                    local flash_len = outdata:match("FLASH_LENGTH=(%d+)")
                    local ram_len = outdata:match("RAM_LENGTH=(%d+)")
                    total_flash_len = tonumber(flash_len) or tonumber(0)
                    total_ram_len = tonumber(ram_len) or tonumber(0)
                end
            end

            if total_flash_real and total_ram_real then
                if total_flash_len > 0 and total_ram_len > 0 then
                    print(string.format("%s flash: %d(%.2f%%%%) ram: %d(%.2f%%%%)",
                        elf,
                        total_flash_real,
                        total_flash_real / total_flash_len * 100,
                        total_ram_real,
                        total_ram_real / total_ram_len * 100))
                else
                    print(string.format("%s flash: %d ram: %d", elf, total_flash_real, total_ram_real))
                end
            else
                print("无法解析size输出")
            end
        end
    end)
rule_end()

rule("bb_table_gen")
    set_extensions(".txt")
    before_build_file(function(target, sourcefile, opt)
        -- imports
        import("core.base.option")
        import("core.theme.theme")
        import("core.project.config")
        import("core.project.depend")
        import("core.tool.compiler")
        import("utils.progress")

        local gen_cx = path.join(target:autogendir(), "rules", "bb_table_gen", path.basename(sourcefile) .. ".c")
        local objectfile = target:objectfile(gen_cx)
        table.insert(target:objectfiles(), objectfile)

        local fileconfig = target:fileconfig(sourcefile)
        local sector_name
        if fileconfig and fileconfig.name then
            sector_name = fileconfig.name
        end

        local pypath = path.join(os.scriptdir(), "gen_baseband_bitrate_table.py")
        local obj_args
        if sector_name then
            obj_args = { pypath, "-i", sourcefile, "-o", gen_cx, "-n", sector_name }
        else
            obj_args = { pypath, "-i", sourcefile, "-o", gen_cx }
        end

        local depargs = { obj_args }

        -- load dependent info
        local dependfile = target:dependfile(objectfile)
        local dependinfo = option.get("rebuild") and {} or (depend.load(dependfile) or {})

        -- need build this object?
        if not depend.is_changed(dependinfo, { lastmtime = os.mtime(objectfile), values = depargs }) then
            return
        end

        -- ensure the source file directory
        os.mkdir(path.directory(gen_cx))

        if sector_name then
            progress.show(opt.progress, "generating baseband bitrate table for %s:  %s -> %s", sector_name, sourcefile, gen_cx)
        else
            progress.show(opt.progress, "generating baseband bitrate table:  %s -> %s", sourcefile, gen_cx)
        end
        os.vrunv("python", obj_args)
        progress.show(opt.progress, "compiling.$(mode) %s -> %s", gen_cx, objectfile)
        compiler.compile(gen_cx, objectfile, { target = target })

        -- update files and values to the dependent file
        dependinfo.values = depargs
        dependinfo.files = { sourcefile }
        depend.save(dependinfo, dependfile)
    end)
rule_end()

rule("open_pwr_tab_gen")
    set_extensions(".csv")
    before_build_file(function(target, sourcefile, opt)
        -- imports
        import("core.base.option")
        import("core.theme.theme")
        import("core.project.config")
        import("core.project.depend")
        import("core.tool.compiler")
        import("utils.progress")

        local gen_cx = path.join(target:autogendir(), "rules", "open_pwr_tab_gen", path.basename(sourcefile) .. ".c")
        local objectfile = target:objectfile(gen_cx)
        table.insert(target:objectfiles(), objectfile)

        local fileconfig = target:fileconfig(sourcefile)
        local sector_name
        if fileconfig and fileconfig.name then
            sector_name = fileconfig.name
        end

        local pypath = path.join(os.scriptdir(), "gen_open_pwr_table.py")
        local obj_args
        if sector_name then
            obj_args = { pypath, "-i", sourcefile, "-o", gen_cx, "-n", sector_name }
        else
            obj_args = { pypath, "-i", sourcefile, "-o", gen_cx }
        end

        local depargs = { obj_args }

        -- load dependent info
        local dependfile = target:dependfile(objectfile)
        local dependinfo = option.get("rebuild") and {} or (depend.load(dependfile) or {})

        -- need build this object?
        if not depend.is_changed(dependinfo, { lastmtime = os.mtime(objectfile), values = depargs }) then
            return
        end

        -- ensure the source file directory
        os.mkdir(path.directory(gen_cx))

        if sector_name then
            progress.show(opt.progress, "generating open loop power table for %s:  %s -> %s", sector_name, sourcefile, gen_cx)
        else
            progress.show(opt.progress, "generating open loop power table:  %s -> %s", sourcefile, gen_cx)
        end
        os.vrunv("python", obj_args)
        progress.show(opt.progress, "compiling.$(mode) %s -> %s", gen_cx, objectfile)
        compiler.compile(gen_cx, objectfile, { target = target })

        -- update files and values to the dependent file
        dependinfo.values = depargs
        dependinfo.files = { sourcefile }
        depend.save(dependinfo, dependfile)
    end)
rule_end()

rule("close_pwr_tab_gen")
    set_extensions(".csv")
    before_build_file(function(target, sourcefile, opt)
        -- imports
        import("core.base.option")
        import("core.theme.theme")
        import("core.project.config")
        import("core.project.depend")
        import("core.tool.compiler")
        import("utils.progress")

        local gen_cx = path.join(target:autogendir(), "rules", "close_pwr_tab_gen", path.basename(sourcefile) .. ".c")
        local objectfile = target:objectfile(gen_cx)
        table.insert(target:objectfiles(), objectfile)

        local fileconfig = target:fileconfig(sourcefile)
        local sector_name
        if fileconfig and fileconfig.name then
            sector_name = fileconfig.name
        end

        local pypath = path.join(os.scriptdir(), "gen_close_pwr_table.py")
        local obj_args
        if sector_name then
            obj_args = { pypath, "-i", sourcefile, "-o", gen_cx, "-n", sector_name }
        else
            obj_args = { pypath, "-i", sourcefile, "-o", gen_cx }
        end

        local depargs = { obj_args }

        -- load dependent info
        local dependfile = target:dependfile(objectfile)
        local dependinfo = option.get("rebuild") and {} or (depend.load(dependfile) or {})

        -- need build this object?
        if not depend.is_changed(dependinfo, { lastmtime = os.mtime(objectfile), values = depargs }) then
            return
        end

        -- ensure the source file directory
        os.mkdir(path.directory(gen_cx))

        if sector_name then
            progress.show(opt.progress, "generating close loop power table for %s:  %s -> %s", sector_name, sourcefile, gen_cx)
        else
            progress.show(opt.progress, "generating close loop power table:  %s -> %s", sourcefile, gen_cx)
        end
        os.vrunv("python", obj_args)
        progress.show(opt.progress, "compiling.$(mode) %s -> %s", gen_cx, objectfile)
        compiler.compile(gen_cx, objectfile, { target = target })

        -- update files and values to the dependent file
        dependinfo.values = depargs
        dependinfo.files = { sourcefile }
        depend.save(dependinfo, dependfile)
    end)
rule_end()

rule("venc_table_gen")
    set_extensions(".json")
    before_build_file(function(target, sourcefile, opt)
        -- imports
        import("core.base.option")
        import("core.theme.theme")
        import("core.project.config")
        import("core.project.depend")
        import("core.tool.compiler")
        import("utils.progress")

        local gen_cx = path.join(target:autogendir(), "rules", "venc_table_gen", path.basename(sourcefile) .. ".c")
        local objectfile = target:objectfile(gen_cx)
        table.insert(target:objectfiles(), objectfile)

        local fileconfig = target:fileconfig(sourcefile)
        local sector_name
        if fileconfig and fileconfig.name then
            sector_name = fileconfig.name
        end

        local pypath = path.join(os.scriptdir(), "gen_tuning_table.py")
        local obj_args
        if sector_name then
            obj_args = { pypath, "-i", sourcefile, "-o", gen_cx, "-n", sector_name }
        else
            obj_args = { pypath, "-i", sourcefile, "-o", gen_cx }
        end

        local depargs = { obj_args }

        -- load dependent info
        local dependfile = target:dependfile(objectfile)
        local dependinfo = option.get("rebuild") and {} or (depend.load(dependfile) or {})

        -- need build this object?
        if not depend.is_changed(dependinfo, { lastmtime = os.mtime(objectfile), values = depargs }) then
            return
        end

        -- ensure the source file directory
        os.mkdir(path.directory(gen_cx))

        if sector_name then
            progress.show(opt.progress, "generating venc tuning table for %s:  %s -> %s", sector_name, sourcefile, gen_cx)
        else
            progress.show(opt.progress, "generating venc tuning table:  %s -> %s", sourcefile, gen_cx)
        end
        os.vrunv("python", obj_args)
        progress.show(opt.progress, "compiling.$(mode) %s -> %s", gen_cx, objectfile)
        compiler.compile(gen_cx, objectfile, { target = target })

        -- update files and values to the dependent file
        dependinfo.values = depargs
        dependinfo.files = { sourcefile }
        depend.save(dependinfo, dependfile)
    end)
rule_end()
