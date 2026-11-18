@echo off
setlocal
if exist "%~dp0.venv\Scripts\python.exe" (
    "%~dp0.venv\Scripts\python.exe" -B -X utf8 "%~dp0tools\open_panel.py"
) else (
    python -B -X utf8 "%~dp0tools\open_panel.py"
)
if errorlevel 1 pause
