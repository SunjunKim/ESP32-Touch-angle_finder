@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" goto ready
where py >nul 2>nul
if errorlevel 1 goto pythonfallback
py -3 -c "import sys; sys.exit(0 if sys.version_info >= (3,10) else 'Python 3.10+ required')"
if errorlevel 1 exit /b 1
py -3 -m venv .venv
goto checkenv
:pythonfallback
python -c "import sys; sys.exit(0 if sys.version_info >= (3,10) else 'Python 3.10+ required')"
if errorlevel 1 exit /b 1
python -m venv .venv
:checkenv
if errorlevel 1 exit /b 1
:ready
.venv\Scripts\python.exe -c "import esptool, serial; import importlib.metadata as m; assert m.version('esptool')=='5.1.0' and m.version('pyserial')=='3.5'" >nul 2>nul
if not errorlevel 1 goto flash
.venv\Scripts\python.exe -m pip install -r requirements.txt
if errorlevel 1 exit /b 1
:flash
.venv\Scripts\python.exe flash_precompiled.py %*
exit /b %errorlevel%
