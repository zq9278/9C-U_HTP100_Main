@echo off
REM ���Ҳ���ֹ OpenOCD ����
tasklist | findstr openocd.exe > nul
if %ERRORLEVEL%==0 (
    echo Terminating OpenOCD process...
    taskkill /IM openocd.exe /F
)


echo All debug tasks terminated.
