@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0readonly_preflight.ps1" %*
exit /b %ERRORLEVEL%
