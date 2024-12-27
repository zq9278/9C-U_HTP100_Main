@echo off
REM 查找并终止 OpenOCD 进程
tasklist | findstr openocd.exe > nul
if %ERRORLEVEL%==0 (
    echo Terminating OpenOCD process...
    taskkill /IM openocd.exe /F
)


echo All debug tasks terminated.
