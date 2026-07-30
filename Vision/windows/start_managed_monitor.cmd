@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_managed_monitor.ps1"
if errorlevel 1 pause
