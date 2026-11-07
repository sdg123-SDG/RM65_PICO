@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_loopback.ps1" %*
exit /b %ERRORLEVEL%
