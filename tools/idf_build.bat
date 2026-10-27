@echo off
REM tools\idf_build.bat - build the firmware with a fully captured log.
REM
REM WHY A .BAT AND NOT POWERSHELL: PowerShell wraps native-tool stderr in
REM NativeCommandError records and truncates multi-line diagnostics, which hid
REM the real CMake error behind its own noise. cmd.exe 2>&1 redirection
REM captures build output verbatim - which is the whole point when the
REM interesting part is a multi-line compiler or CMake message.
REM
REM ALSO: prepends the real Python 3.11. The Windows Store Python 3.12 that
REM sits on the default PATH breaks ESP-IDF's venv lookup and produces a
REM misleading "virtual environment is corrupted" error.
REM See docs/08-firmware-setup.md.
REM
REM Usage:  tools\idf_build.bat [idf.py-subcommand]    (default: build)

setlocal enabledelayedexpansion

REM Defaults are the standard per-user install locations; set PY311 and/or
REM IDF_PATH in the environment beforehand to point somewhere else.
if not defined PY311 set "PY311=%LOCALAPPDATA%\Programs\Python\Python311"
if not defined IDF_PATH set "IDF_PATH=%USERPROFILE%\esp\esp-idf"
set "PATH=%PY311%;%PY311%\Scripts;%PATH%"

set "SUBCMD=%~1"
if "%SUBCMD%"=="" set "SUBCMD=build"

REM ESP-IDF's export.bat refuses to run unless PROMPT is defined - that is how
REM it detects "am I in a real cmd.exe shell?". When cmd.exe is launched
REM non-interactively (e.g. `cmd /c` from Git Bash or from a tool), PROMPT is
REM NOT set, and export.bat bails out with "This .bat file is for Windows
REM CMD.EXE shell only." while still returning errorlevel 0 - so it fails
REM silently and the only symptom is idf.py missing afterwards.
REM export.bat's actual guard is `if defined MSYSTEM`. Git Bash / MSYS2 sets
REM MSYSTEM and cmd.exe INHERITS it, so launching this from a Git Bash shell
REM makes export.bat think it is running under MSYS and bail out - while still
REM returning errorlevel 0, so it fails silently. Clear it for this process.
set "MSYSTEM="
if not defined PROMPT set "PROMPT=$P$G"

echo [idf_build] IDF_PATH=%IDF_PATH%
if not exist "%IDF_PATH%\export.bat" (
  echo [idf_build] FATAL: export.bat not found at %IDF_PATH%
  exit /b 1
)

echo [idf_build] sourcing export.bat ...
call "%IDF_PATH%\export.bat"
echo [idf_build] export.bat returned %errorlevel%

where idf.py >nul 2>&1
if errorlevel 1 (
  echo [idf_build] FATAL: idf.py still not on PATH after export.bat
  exit /b 1
)

REM Project directory: the previous board's flight shell (ESP-FLY, ESP-IDF v5.3)
REM unless a wrapper points elsewhere (tools\lens_test\build.bat sets IDF_PROJECT
REM to its ESP32-CAM bench project). The live S3-Zero firmware builds with
REM scripts\idf.bat instead (ESP-IDF v5.5.5).
if not defined IDF_PROJECT set "IDF_PROJECT=%~dp0..\firmware\projects\espfly"
cd /d "%IDF_PROJECT%"
echo [idf_build] === idf.py %SUBCMD% in %CD% ===
call idf.py %SUBCMD%
set "RC=%errorlevel%"
echo [idf_build] idf.py exited %RC%
exit /b %RC%
