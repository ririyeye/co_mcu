set XMAKE_GLOBALDIR=%~dp0..
echo %XMAKE_GLOBALDIR%
cd /D %XMAKE_GLOBALDIR%

openocd -f bsp/Firmware/gd32e11_script/openocd_gdlink.cfg -f bsp/Firmware/gd32e11_script/openocd_remote.cfg
pause
