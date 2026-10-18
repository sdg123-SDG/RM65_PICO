@echo off
setlocal
if not "%~1"=="" goto run
echo RM65_PICO - MuJoCo simulation
echo [T] Teleoperation   [J] Joystick   [S] Service   [D] Diagnostics   [V] Scene only
choice /C TJSDV /N /M "Select mode [T/J/S/D/V]: "
if errorlevel 5 set "MODE=scene"
if errorlevel 4 if not defined MODE set "MODE=diagnostics"
if errorlevel 3 if not defined MODE set "MODE=service"
if errorlevel 2 if not defined MODE set "MODE=jog"
if errorlevel 1 if not defined MODE set "MODE=teleop"
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0start.ps1" -Mode "%MODE%"
goto finish
:run
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0start.ps1" %*
:finish
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" pause
exit /b %RESULT%
