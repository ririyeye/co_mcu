set XMAKE_GLOBALDIR=%~dp0..
echo %XMAKE_GLOBALDIR%
cd /D %XMAKE_GLOBALDIR%

@RD /S /Q .xmake
@RD /S /Q build
@RD /S /Q install
@RD /S /Q .cache

pause
