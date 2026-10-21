@echo off
chcp 65001 >nul
cd /d "%~dp0"
".venv\Scripts\python.exe" -B ring_digit.py --camera 1
pause
