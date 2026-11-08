@echo off
"E:\SDG-VLA\RM65_PICO\.venv\Scripts\python.exe" -u "%~dp0start_collect.py" %*
set "collect_exit=%errorlevel%"
pause
exit /b %collect_exit%
