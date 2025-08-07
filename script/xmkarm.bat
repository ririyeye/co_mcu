
set XMAKE_GLOBALDIR=%~dp0..
echo %XMAKE_GLOBALDIR%
cd /D %XMAKE_GLOBALDIR%

xmake g --network=private
xmake.exe f -p cross ^
    --toolchain=m4-arm-none-eabi ^
    -y -vD ^
    -m minsizerel ^
    -P .
xmake.exe -D -P .
xmake.exe install -o install -P .
pause
