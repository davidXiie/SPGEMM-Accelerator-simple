@echo off
REM Windows batch runner for tb_pe_top + Cocotb + Icarus Verilog.
REM Usage:
REM   run_comp.bat
REM
REM Optional (edit inline or set before running):
REM   set MODULE=test_comp
REM   set TESTCASE=test_comp_case1_p0
REM   set TOPLEVEL=tb_pe_top

set PYTHONIOENCODING=utf-8
set PATH=C:\Users\Administrator\.conda\envs\gcnenv;C:\Users\Administrator\.conda\envs\gcnenv\Scripts;C:\iverilog\bin;C:\iverilog\gtkwave\bin;%PATH%
cd /d "d:\BaiduSyncdisk\PROJECT\SPGEMM-Accelerator-simple\test\pe_sim"

set COCOTB_LIB=C:\Users\Administrator\.conda\envs\gcnenv\Lib\site-packages\cocotb\libs
set PROJ_ROOT=d:\BaiduSyncdisk\PROJECT\SPGEMM-Accelerator-simple

set MODULE=test_comp
set COCOTB_TESTCASE=
set COCOTB_TEST_FILTER=test_comp_case1_p0
set TOPLEVEL=tb_pe_top
set COCOTB_LOG_LEVEL=INFO

echo ========================================
echo [1/2] Compiling PE testbench...
echo ========================================
if not exist sim_build mkdir sim_build

iverilog -g2012 -DCOCOTB_SIM=1 -I%PROJ_ROOT%\rtl\include ^
    -s %TOPLEVEL% ^
    -o sim_build\sim.vvp ^
    %PROJ_ROOT%\rtl\sim\tb_pe_top.v ^
    %PROJ_ROOT%\rtl\core\pe_top.v ^
    %PROJ_ROOT%\rtl\core\pe_mul_array.v ^
    %PROJ_ROOT%\rtl\core\accum_bank.v ^
    %PROJ_ROOT%\rtl\core\row_accumulator_4bank.v ^
    %PROJ_ROOT%\rtl\infrastructure\scratchpad.v

if %ERRORLEVEL% NEQ 0 (
    echo [FAIL] Compile error
    exit /b 1
)
echo [OK] Compile passed.

echo.
echo ========================================
echo [2/2] Running Cocotb test...
echo ========================================
echo MODULE=%MODULE%
echo COCOTB_TEST_FILTER=%COCOTB_TEST_FILTER%
echo TOPLEVEL=%TOPLEVEL%

vvp -M %COCOTB_LIB% -m cocotbvpi_icarus sim_build\sim.vvp

echo.
echo ========================================
echo Done.
echo ========================================
