# Host-side regression tests for the push/event protocol.
#
#   powershell -ExecutionPolicy Bypass -File run_tests.ps1
#
# 1. test_beam.c          - compiles the REAL beam.c with stubs and drives it
#                           with the change request's acceptance waveforms.
#                           Run twice: once with beam.c's compiled-in default
#                           (BEAM_AXLE_PATTERN_ENABLE=0, this install's real
#                           setting) and once with it forced back on, so the
#                           axle-pair code path stays covered too.
# 2. test_esp_parsers.cpp - compiles the bridge's message parsers, sliced
#                           verbatim out of src/main.cpp by gen_esp_test.py.
# 3. test_wire_contract.py- reads both firmwares and checks that every wire key
#                           one side emits is the one the other side parses.
$ErrorActionPreference = "Stop"
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { throw "Visual Studio Build Tools (MSVC) not found - vswhere.exe is missing" }
$vsdir = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
$vc  = Join-Path $vsdir "VC\Auxiliary\Build\vcvars64.bat"
$dir = $PSScriptRoot
$rc  = 0

Write-Host "=== 1a/3  beam.c (default: axle pattern OFF) ================" -ForegroundColor Cyan
cmd /c "`"$vc`" >nul 2>&1 && cd /d `"$dir`" && cl /nologo /W3 /I..\msp430-gate-controller test_beam.c ..\msp430-gate-controller\beam.c /Fe:test_beam.exe >build_beam.log 2>&1 && .\test_beam.exe"
if ($LASTEXITCODE -ne 0) { $rc = 1 }

Write-Host "`n=== 1b/3  beam.c (axle pattern ON, coverage only) ===========" -ForegroundColor Cyan
cmd /c "`"$vc`" >nul 2>&1 && cd /d `"$dir`" && cl /nologo /W3 /I..\msp430-gate-controller /DBEAM_AXLE_PATTERN_ENABLE=1 test_beam.c ..\msp430-gate-controller\beam.c /Fe:test_beam_axle.exe >build_beam_axle.log 2>&1 && .\test_beam_axle.exe"
if ($LASTEXITCODE -ne 0) { $rc = 1 }

Write-Host "`n=== 2/3  ESP32 bridge parsers ==============================" -ForegroundColor Cyan
python "$dir\gen_esp_test.py"
cmd /c "`"$vc`" >nul 2>&1 && cd /d `"$dir`" && cl /nologo /W3 /EHsc /D_CRT_SECURE_NO_WARNINGS test_esp_parsers.cpp /Fe:test_esp.exe >build_esp.log 2>&1 && .\test_esp.exe"
if ($LASTEXITCODE -ne 0) { $rc = 1 }

Write-Host "`n=== 3/3  MSP430 <-> ESP32 wire contract ====================" -ForegroundColor Cyan
python "$dir\test_wire_contract.py"
if ($LASTEXITCODE -ne 0) { $rc = 1 }

Write-Host ""
if ($rc -eq 0) { Write-Host "ALL SUITES PASSED" -ForegroundColor Green }
else           { Write-Host "SOME SUITES FAILED" -ForegroundColor Red }
exit $rc
