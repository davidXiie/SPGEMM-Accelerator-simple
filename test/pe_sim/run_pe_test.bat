@echo off
set PYTHONIOENCODING=utf-8
set PATH=C:\Users\Administrator\.conda\envs\gcnenv;C:\Users\Administrator\.conda\envs\gcnenv\Scripts;C:\iverilog\bin;C:\iverilog\gtkwave\bin;%PATH%

cd /d "d:\BaiduSyncdisk\PROJECT\SPGEMM-Accelerator-simple\test\pe_sim"

set COCOTB_LIB=C:\Users\Administrator\.conda\envs\gcnenv\Lib\site-packages\cocotb\libs
set PROJ_ROOT=d:\BaiduSyncdisk\PROJECT\SPGEMM-Accelerator-simple

echo ========================================
echo [1/2] Compiling PE testbench...
echo ========================================
if not exist sim_build mkdir sim_build

iverilog -g2012 -DCOCOTB_SIM=1 -I%PROJ_ROOT%\rtl\include ^
    -s tb_pe_top ^
    -o sim_build\sim.vvp ^
    %PROJ_ROOT%\rtl\sim\tb_pe_top.v ^
    %PROJ_ROOT%\rtl\core\pe_top.v ^
    %PROJ_ROOT%\rtl\core\pe_task_packer.v ^
    %PROJ_ROOT%\rtl\core\pe_serializer.v ^
    %PROJ_ROOT%\rtl\core\pe_accumulator.v ^
    %PROJ_ROOT%\rtl\core\pe_mul_array.v ^
    %PROJ_ROOT%\rtl\infrastructure\scratchpad.v

if %ERRORLEVEL% NEQ 0 (
    echo [FAIL] Compile error
    exit /b 1
)
echo [OK] Compile passed.

echo.
echo ========================================
echo [2/2] Running cocotb test...
echo ========================================
set MODULE=test_pe
set TESTCASE=test_pe_50x50
set TOPLEVEL=tb_pe_top
set COCOTB_LOG_LEVEL=INFO

vvp -M %COCOTB_LIB% -m cocotbvpi_icarus sim_build/sim.vvp

echo.
echo ========================================
echo Done.
echo ========================================
