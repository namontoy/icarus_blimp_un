@echo off
REM scripts\idf.bat - run idf.py for one firmware project on Windows.
REM
REM   scripts\idf.bat i2c_tools build
REM   scripts\idf.bat i2c_tools -p COM5 flash monitor
REM   scripts\idf.bat optflow_cam set-target esp32
REM
REM Why this exists: ESP-IDF's export.bat refuses to run when the MSYSTEM
REM variable is set (Git Bash sets it, and cmd.exe inherits it) and when
REM PROMPT is undefined (any non-interactive cmd.exe), and it fails silently
REM in both cases. The Windows Store Python 3.12 on the default PATH also
REM breaks ESP-IDF's virtualenv lookup. This script clears all three traps,
REM then runs idf.py with whatever arguments follow the project name.
REM
REM Defaults are the standard per-user locations. Set IDF_PATH and PY311 in
REM the environment first if yours differ. The firmware was written against
REM ESP-IDF v5.5.5.

setlocal enabledelayedexpansion

if "%~1"=="" (
  echo usage: scripts\idf.bat ^<project^> ^<idf.py arguments...^>
  echo        projects: i2c_tools  optflow_cam  blink
  exit /b 1
)
set "PROJECT=%~1"
REM shift /1 keeps %0: a plain shift moves it too, and %~dp0 below would then
REM name the first argument's folder instead of this script's.
shift /1

if not defined PY311 set "PY311=%LOCALAPPDATA%\Programs\Python\Python311"
if not defined IDF_PATH set "IDF_PATH=%USERPROFILE%\esp\esp-idf-v5.5.5"
set "PATH=%PY311%;%PY311%\Scripts;%PATH%"
set "MSYSTEM="
if not defined PROMPT set "PROMPT=$P$G"

if not exist "%IDF_PATH%\export.bat" (
  echo [idf] ESP-IDF not found at %IDF_PATH% - set IDF_PATH
  exit /b 1
)
call "%IDF_PATH%\export.bat" >nul
where idf.py >nul 2>&1
if errorlevel 1 (
  echo [idf] idf.py not on PATH after export.bat - is the Python env installed? ^(install.ps1 esp32s3^)
  exit /b 1
)

cd /d "%~dp0..\firmware\projects\%PROJECT%" || exit /b 1
set "ARGS="
:collect
if "%~1"=="" goto run
set "ARGS=!ARGS! %1"
shift /1
goto collect
:run
if "!ARGS!"=="" set "ARGS= build"
echo [idf] idf.py!ARGS!   (in %CD%, IDF_PATH=%IDF_PATH%)
call idf.py!ARGS!
exit /b %errorlevel%
