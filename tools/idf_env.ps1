# tools/idf_env.ps1 - enter the ESP-IDF build environment on this machine.
#
# WHY THIS EXISTS: this machine has TWO Pythons. `python` on the default PATH
# is the Windows Store build (3.12), whose sandboxed per-user redirection
# breaks ESP-IDF's virtualenv bootstrap. ESP-IDF was therefore installed
# against the real Python 3.11 at
#   %LOCALAPPDATA%\Programs\Python\Python311
# and its venv is named after that version: .espressif\python_env\idf5.3_py3.11_env
#
# export.ps1 picks its venv from whichever `python` it finds FIRST on PATH.
# Dot-sourcing it without fixing PATH makes it look for a py3.12 venv that
# does not exist, and idf.py then fails with a misleading
# "virtual environment ... is corrupted" message that has nothing to do with
# the project. Prepending 3.11 is the whole fix.
#
# USAGE (must be DOT-SOURCED so the environment persists in your shell):
#     . .\tools\idf_env.ps1
#     cd firmware\projects\espfly      # the previous board's shell (ESP-IDF v5.3)
#     idf.py build

$ErrorActionPreference = "Stop"

# Defaults are the standard per-user install locations; set $env:PY311 and/or
# $env:IDF_PATH beforehand to point somewhere else.
$Py311 = if ($env:PY311) { $env:PY311 } else { "$env:LOCALAPPDATA\Programs\Python\Python311" }
$IdfPath = if ($env:IDF_PATH) { $env:IDF_PATH } else { "$env:USERPROFILE\esp\esp-idf" }

if (-not (Test-Path "$Py311\python.exe")) {
    Write-Error "Python 3.11 not found at $Py311 - update this script or reinstall ESP-IDF."
}
if (-not (Test-Path "$IdfPath\export.ps1")) {
    Write-Error "ESP-IDF not found at $IdfPath - see docs/08-firmware-setup.md."
}

$env:PATH = "$Py311;$Py311\Scripts;" + $env:PATH
$env:IDF_PATH = $IdfPath

Write-Host "python : $((Get-Command python).Source)"
Write-Host "IDF    : $env:IDF_PATH"

. "$IdfPath\export.ps1"
