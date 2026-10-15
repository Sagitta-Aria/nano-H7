@echo off
rem Double-click launcher for the web console. Keeps its window open on failure.
cd /d "%~dp0.."
python -m host.webserver %*
if errorlevel 1 pause
