@echo off
REM tools\lens_test\build.bat - build or flash the ESP32-CAM lens-test firmware.
REM
REM Same PATH workarounds as the flight firmware (tools\idf_build.bat), pointed at
REM this project instead. Multi-word idf.py commands go in one quoted argument.
REM
REM Usage:  tools\lens_test\build.bat                   (build)
REM         tools\lens_test\build.bat "-p COM5 flash"
REM         tools\lens_test\build.bat menuconfig          (Wi-Fi mode, SSID, password)
REM
REM `monitor` is interactive, so run it from your own terminal instead:
REM         . .\tools\idf_env.ps1 ; cd tools\lens_test\esp32cam ; idf.py -p COM5 monitor

set "IDF_PROJECT=%~dp0esp32cam"
call "%~dp0..\idf_build.bat" %*
exit /b %errorlevel%
