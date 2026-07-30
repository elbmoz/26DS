@echo off
setlocal
cd /d "%~dp0"
python stream_receiver.py
set "receiver_exit=%errorlevel%"
if not "%receiver_exit%"=="0" pause
exit /b %receiver_exit%
