@echo off
cd /d "%~dp0.."
python -m host.gui
if errorlevel 1 pause
