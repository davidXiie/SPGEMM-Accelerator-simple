@echo off
setlocal enabledelayedexpansion
set PATH=C:\iverilog\bin;C:\Users\Administrator\.conda\envs\gcnenv;C:\Users\Administrator\.conda\envs\gcnenv\Scripts;C:\Users\Administrator\.conda\envs\gcnenv\Library\bin;%PATH%

set COCOTB_TEST_MODULES=test
set COCOTB_TESTCASE=test_mmap
set COCOTB_TOPLEVEL=tb_mmap
set COCOTB_LOG_LEVEL=WARNING
set COCOTB_SIM=1
set PYTHONIOENCODING=utf-8
set PYTHONPATH=%~dp0

cd /d %~dp0

for /f "delims=" %%i in ('cocotb-config --lib-dir')            do set COCOTB_LIB=%%i
for /f "delims=" %%i in ('cocotb-config --lib-name vpi icarus') do set COCOTB_VPI_MODULE=%%i
for /f "delims=" %%i in ('cocotb-config --libpython')           do set LIBPYTHON_LOC=%%i
for /f "delims=" %%i in ('cocotb-config --python-bin')          do set PYGPI_PYTHON_BIN=%%i
for %%F in ("%LIBPYTHON_LOC%") do set LIBPYTHON_DIR=%%~dpF
set PATH=%LIBPYTHON_DIR%;%PATH%

echo ========================================
echo [1/2] Compiling mmap-DDR accelerator...
echo ========================================
if exist sim_build rmdir /s /q sim_build
mkdir sim_build

iverilog -g2012 ^
    -DCOCOTB_SIM=1 ^
    -I..\..\rtl\include ^
    -s tb_mmap ^
    -o sim_build\sim_mmap.vvp ^
    ..\mmap_sim\tb_mmap.v ^
    ..\..\rtl\core\accelerator_axi_top.v ^
    ..\..\rtl\core\axi_loader.v ^
    ..\..\rtl\core\axi_c_drain.v ^
    ..\..\rtl\core\pe_cluster.v ^
    ..\..\rtl\core\pe_top.v ^
    ..\..\rtl\core\pe_mul_array.v ^
    ..\..\rtl\core\fp16_mul.v ^
    ..\..\rtl\core\fp16_add.v ^
    ..\..\rtl\core\accum_bank_16.v ^
    ..\..\rtl\core\row_accumulator_16bank.v ^
    ..\..\rtl\infrastructure\scratchpad.v

if %ERRORLEVEL% NEQ 0 (echo [FAIL] Compile error & exit /b 1)
echo [OK] Compile passed.

echo.
echo ========================================
echo [2/2] Running automated tests...
echo ========================================

python runner.py

echo ========================================
echo Done.
echo ========================================
