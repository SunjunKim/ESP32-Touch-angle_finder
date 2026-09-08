@echo off
setlocal
where py >nul 2>nul
if not errorlevel 1 (
  py -3 "%~dp0update_firmware.py" %*
  exit /b
)
where python >nul 2>nul
if not errorlevel 1 (
  python "%~dp0update_firmware.py" %*
  exit /b
)
echo Python 3.9+ is required. Install Python with the launcher or add it to PATH.
exit /b 1
