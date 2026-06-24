@echo off
::=============================================================================
:: Windows runner for 4-PE cluster simulation + Cocotb 2.x + Icarus Verilog.
::
:: Usage:
::   conda activate gcnenv
::   run_cluster.bat
::
:: Optional (set before running):
::   COCOTB_TESTCASE=test_comp_case1_cluster   (default)
::=============================================================================

set PYTHONIOENCODING=utf-8
set PATH=C:\Users\Administrator\.conda\envs\gcnenv;C:\Users\Administrator\.conda\envs\gcnenv\Scripts;C:\iverilog\bin;C:\iverilog\gtkwave\bin;%PATH%
cd /d "%~dp0"

:: --- Cocotb paths ---
set COCOTB_ENV=C:\Users\Administrator\.conda\envs\gcnenv
set COCOTB_LIB=%COCOTB_ENV%\Lib\site-packages\cocotb\libs
set PYGPI_PYTHON_BIN=%COCOTB_ENV%\python.exe
set LIBPYTHON_LOC=%COCOTB_ENV%\python311.dll

if not exist "%COCOTB_LIB%" (
    echo [FAIL] Cocotb library not found: %COCOTB_LIB%
    exit /b 1
)
if not exist "%PYGPI_PYTHON_BIN%" (
    echo [FAIL] Python not found: %PYGPI_PYTHON_BIN%
    exit /b 1
)

:: --- Test configuration ---
if not defined COCOTB_TEST_MODULES   set COCOTB_TEST_MODULES=test_comp
if not defined COCOTB_TESTCASE       set COCOTB_TESTCASE=test_comp_case1_cluster
set TOPLEVEL=tb_pe_cluster
set COCOTB_LOG_LEVEL=INFO
set PYTHONPATH=%~dp0;%PYTHONPATH%

:: --- Project root (relative to this script: ..\..) ---
pushd "%~dp0..\.."
set PROJ_ROOT=%CD%
popd

echo ========================================
echo [1/2] Compiling 4-PE cluster...
echo ========================================
if not exist sim_build mkdir sim_build

iverilog -g2012 -DCOCOTB_SIM=1 -I"%PROJ_ROOT%\rtl\include" ^
    -s %TOPLEVEL% ^
    -o sim_build\sim_cluster.vvp ^
    "%PROJ_ROOT%\rtl\sim\tb_pe_cluster.v" ^
    "%PROJ_ROOT%\rtl\core\pe_cluster.v" ^
    "%PROJ_ROOT%\rtl\core\pe_top.v" ^
    "%PROJ_ROOT%\rtl\core\pe_task_packer.v" ^
    "%PROJ_ROOT%\rtl\core\pe_mul_array.v" ^
    "%PROJ_ROOT%\rtl\core\fp16_mul.v" ^
    "%PROJ_ROOT%\rtl\core\fp32_add.v" ^
    "%PROJ_ROOT%\rtl\core\accum_bank.v" ^
    "%PROJ_ROOT%\rtl\core\row_accumulator_4bank.v" ^
    "%PROJ_ROOT%\rtl\infrastructure\scratchpad.v"

if %ERRORLEVEL% NEQ 0 (
    echo [FAIL] Compile error
    exit /b 1
)
echo [OK] Compile passed.

echo.
echo ========================================
echo [2/2] Running Cocotb cluster test...
echo ========================================
echo COCOTB_TEST_MODULES=%COCOTB_TEST_MODULES%
echo COCOTB_TESTCASE=%COCOTB_TESTCASE%
echo TOPLEVEL=%TOPLEVEL%
echo.

vvp -M "%COCOTB_LIB%" -m cocotbvpi_icarus sim_build\sim_cluster.vvp

echo.
echo ========================================
echo Done.
echo ========================================
