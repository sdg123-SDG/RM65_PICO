@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_real_trial.ps1" %*
exit /b %ERRORLEVEL%
