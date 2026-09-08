@echo off
setlocal
powershell.exe -NoProfile -File "%~dp0update.ps1" %*
exit /b %errorlevel%
