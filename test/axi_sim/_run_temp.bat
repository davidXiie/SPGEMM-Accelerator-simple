@echo off
setlocal enabledelayedexpansion
set PATH=C:\iverilog\bin;C:\Users\Administrator\.conda\envs\gcnenv;C:\Users\Administrator\.conda\envs\gcnenv\Scripts;C:\Users\Administrator\.conda\envs\gcnenv\Library\bin;%PATH%

set COCOTB_TEST_MODULES=test_accelerator_axi
set COCOTB_TESTCASE=test_axi_case1
set COCOTB_TOPLEVEL=tb_accelerator_axi
set COCOTB_LOG_LEVEL=INFO
set COCOTB_SIM=1
set PYTHONIOENCODING=utf-8
set PYTHONPATH=d:\BaiduSyncdisk\PROJECT\SPGEMM-Accelerator-simple\test\axi_sim

cd /d d:\BaiduSyncdisk\PROJECT\SPGEMM-Accelerator-simple\test\axi_sim

for /f "delims=" %%i in ('cocotb-config --lib-dir')            do set COCOTB_LIB=%%i
for /f "delims=" %%i in ('cocotb-config --lib-name vpi icarus') do set COCOTB_VPI_MODULE=%%i
for /f "delims=" %%i in ('cocotb-config --libpython')           do set LIBPYTHON_LOC=%%i
for /f "delims=" %%i in ('cocotb-config --python-bin')          do set PYGPI_PYTHON_BIN=%%i
for %%F in ("%LIBPYTHON_LOC%") do set LIBPYTHON_DIR=%%~dpF
set PATH=%LIBPYTHON_DIR%;%PATH%

echo ========================================
echo Running AXI accelerator test...
echo ========================================

vvp -M "%COCOTB_LIB%" -m %COCOTB_VPI_MODULE% sim_build/sim_axi.vvp

echo ========================================
echo Done (exit: %ERRORLEVEL%)
echo ========================================
