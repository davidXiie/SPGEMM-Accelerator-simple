@echo off
::
:: Windows runner for tb_core_top_1pe + Cocotb + Icarus Verilog.
:: Tests full DDR → PE → DDR flow with AXI DDR model.
::
:: Usage:
::   conda activate gcnenv
::   run_core_top_1pe.bat

setlocal enabledelayedexpansion

rem --- cocotb test configuration ---
set COCOTB_TEST_MODULES=test_core_top_1pe
set COCOTB_TESTCASE=test_core_top_1pe_small
set COCOTB_TOPLEVEL=tb_core_top_1pe
set COCOTB_LOG_LEVEL=INFO
set COCOTB_SIM=1
set PYTHONIOENCODING=utf-8

rem --- project root ---
if not defined PROJ_ROOT set PROJ_ROOT=d:\BaiduSyncdisk\PROJECT\SPGEMM-Accelerator-simple

rem --- ensure this script's directory is on Python path ---
set PYTHONPATH=%~dp0;%PYTHONPATH%

rem --- check required tools ---
where iverilog >nul 2>&1 || (echo [FAIL] iverilog not found in PATH. & exit /b 1)
where vvp      >nul 2>&1 || (echo [FAIL] vvp not found in PATH. & exit /b 1)
where cocotb-config >nul 2>&1 || (
    echo [FAIL] cocotb-config not found in PATH.
    echo        Activate your Conda environment, e.g.: conda activate gcnenv
    exit /b 1
)

rem --- auto-detect cocotb paths ---
for /f "delims=" %%i in ('cocotb-config --lib-dir')            do set COCOTB_LIB=%%i
for /f "delims=" %%i in ('cocotb-config --lib-name vpi icarus') do set COCOTB_VPI_MODULE=%%i
for /f "delims=" %%i in ('cocotb-config --libpython')           do set LIBPYTHON_LOC=%%i
for /f "delims=" %%i in ('cocotb-config --python-bin')          do set PYGPI_PYTHON_BIN=%%i

if not exist "%COCOTB_LIB%" (echo [FAIL] Cocotb library dir not found & exit /b 1)
if not exist "%LIBPYTHON_LOC%" (echo [FAIL] libpython not found & exit /b 1)

for %%F in ("%LIBPYTHON_LOC%") do set LIBPYTHON_DIR=%%~dpF
set PATH=%LIBPYTHON_DIR%;%PATH%

cd /d "%~dp0"

echo ========================================
echo [1/2] Compiling core_top_1pe testbench...
echo ========================================
if not exist sim_build mkdir sim_build

iverilog -g2012 ^
    -DCOCOTB_SIM=1 ^
    -I"%PROJ_ROOT%/rtl/include" ^
    -s %COCOTB_TOPLEVEL% ^
    -o sim_build/sim_core_top_1pe.vvp ^
    "%PROJ_ROOT%/test/pe_sim/tb_core_top_1pe.v" ^
    "%PROJ_ROOT%/rtl/core/core_top_1pe.v" ^
    "%PROJ_ROOT%/rtl/core/descriptor_loader.v" ^
    "%PROJ_ROOT%/rtl/core/a_group_loader.v" ^
    "%PROJ_ROOT%/rtl/core/b_broadcast_loader.v" ^
    "%PROJ_ROOT%/rtl/core/pe_top.v" ^
    "%PROJ_ROOT%/rtl/core/pe_mul_array.v" ^
    "%PROJ_ROOT%/rtl/core/fp16_mul.v" ^
    "%PROJ_ROOT%/rtl/core/fp16_add.v" ^
    "%PROJ_ROOT%/rtl/core/accum_bank.v" ^
    "%PROJ_ROOT%/rtl/core/row_accumulator_4bank.v" ^
    "%PROJ_ROOT%/rtl/core/pe_c_to_densebuf.v" ^
    "%PROJ_ROOT%/rtl/core/c_dense_buffer.v" ^
    "%PROJ_ROOT%/rtl/core/c_dense_ddr_writer.v" ^
    "%PROJ_ROOT%/rtl/infrastructure/scratchpad.v"

if %ERRORLEVEL% NEQ 0 (
    echo [FAIL] Compile error
    exit /b 1
)
echo [OK] Compile passed.

echo.
echo ========================================
echo [2/2] Running Cocotb core_top_1pe test...
echo ========================================
echo COCOTB_TEST_MODULES=%COCOTB_TEST_MODULES%
echo COCOTB_TESTCASE=%COCOTB_TESTCASE%
echo COCOTB_TOPLEVEL=%COCOTB_TOPLEVEL%
echo.

vvp ^
    -M "%COCOTB_LIB%" ^
    -m %COCOTB_VPI_MODULE% ^
    sim_build/sim_core_top_1pe.vvp

echo.
echo ========================================
echo Done.
echo ========================================
