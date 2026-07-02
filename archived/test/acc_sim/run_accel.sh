#!/usr/bin/env bash
#
# Cocotb full-accelerator simulation.
# Usage: conda activate py311 && bash run_accel.sh
# Optional: COCOTB_TESTCASE=test_accel_case1 bash run_accel.sh

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PROJ_ROOT="${PROJ_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"

export COCOTB_TEST_MODULES="${COCOTB_TEST_MODULES:-test_accelerator}"
export COCOTB_TESTCASE="${COCOTB_TESTCASE:-test_accel_case1}"
export COCOTB_TOPLEVEL="${COCOTB_TOPLEVEL:-tb_accelerator}"
export COCOTB_LOG_LEVEL="${COCOTB_LOG_LEVEL:-INFO}"
export COCOTB_SIM=1
export PYTHONIOENCODING="${PYTHONIOENCODING:-utf-8}"
export PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"

for tool in iverilog vvp cocotb-config; do
    command -v "$tool" >/dev/null 2>&1 || { echo "[FAIL] $tool not found"; exit 1; }
done

COCOTB_LIB="$(cocotb-config --lib-dir)"
COCOTB_VPI_MODULE="$(cocotb-config --lib-name vpi icarus)"
LIBPYTHON_LOC="$(cocotb-config --libpython)"
PYGPI_PYTHON_BIN="$(cocotb-config --python-bin)"
export LIBPYTHON_LOC PYGPI_PYTHON_BIN
export LD_LIBRARY_PATH="$(dirname "$LIBPYTHON_LOC")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

echo "========================================"
echo "[1/2] Compiling full accelerator..."
echo "========================================"

mkdir -p sim_build

iverilog -g2012 \
    -DCOCOTB_SIM=1 \
    -DC_ROW_ADDR_BITS="${C_ROW_ADDR_BITS:-8}" \
    -I"$PROJ_ROOT/rtl/include" \
    -s "$COCOTB_TOPLEVEL" \
    -o sim_build/sim_accel.vvp \
    "$PROJ_ROOT/rtl/sim/tb_accelerator.v" \
    "$PROJ_ROOT/rtl/core/accelerator_top.v" \
    "$PROJ_ROOT/rtl/core/pe_load_ctrl.v" \
    "$PROJ_ROOT/rtl/core/pe_drain_ctrl.v" \
    "$PROJ_ROOT/rtl/core/a_global_buffer.v" \
    "$PROJ_ROOT/rtl/core/b_global_buffer.v" \
    "$PROJ_ROOT/rtl/core/c_global_buffer.v" \
    "$PROJ_ROOT/rtl/core/pe_cluster.v" \
    "$PROJ_ROOT/rtl/core/pe_top.v" \
    "$PROJ_ROOT/rtl/core/pe_mul_array.v" \
    "$PROJ_ROOT/rtl/core/fp16_mul.v" \
    "$PROJ_ROOT/rtl/core/fp16_add.v" \
    "$PROJ_ROOT/rtl/core/accum_bank.v" \
    "$PROJ_ROOT/rtl/core/accum_bank_16.v" \
    "$PROJ_ROOT/rtl/core/row_accumulator_16bank.v" \
    "$PROJ_ROOT/rtl/infrastructure/scratchpad.v"

if [ $? -ne 0 ]; then echo "[FAIL] Compile error"; exit 1; fi
echo "[OK] Compile passed."

echo
echo "========================================"
echo "[2/2] Running Cocotb accelerator test..."
echo "========================================"
echo "COCOTB_TEST_MODULES=$COCOTB_TEST_MODULES"
echo "COCOTB_TESTCASE=$COCOTB_TESTCASE"
echo "COCOTB_TOPLEVEL=$COCOTB_TOPLEVEL"
echo

vvp \
    -M "$COCOTB_LIB" \
    -m "$COCOTB_VPI_MODULE" \
    sim_build/sim_accel.vvp

echo
echo "========================================"
echo "Done."
echo "========================================"
