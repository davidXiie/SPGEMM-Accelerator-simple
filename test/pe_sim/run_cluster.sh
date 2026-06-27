#!/usr/bin/env bash
# Run 4-PE cluster simulation.
# Usage: conda activate py311 && bash run_cluster.sh

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PROJ_ROOT="${PROJ_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"

export COCOTB_TEST_MODULES="${COCOTB_TEST_MODULES:-test_comp}"
export COCOTB_TESTCASE="${COCOTB_TESTCASE:-test_comp_case1_cluster}"
export COCOTB_TOPLEVEL="${COCOTB_TOPLEVEL:-tb_pe_cluster}"
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
echo "[1/2] Compiling 4-PE cluster..."
echo "========================================"

mkdir -p sim_build

iverilog -g2012 \
    -DCOCOTB_SIM=1 \
    -DSIMULATION \
    -DC_ROW_ADDR_BITS="${C_ROW_ADDR_BITS:-7}" \
    -I"$PROJ_ROOT/rtl/include" \
    -s "$COCOTB_TOPLEVEL" \
    -o sim_build/sim_cluster.vvp \
    "$PROJ_ROOT/rtl/sim/tb_pe_cluster.v" \
    "$PROJ_ROOT/rtl/core/pe_cluster.v" \
    "$PROJ_ROOT/rtl/core/pe_top.v" \
    "$PROJ_ROOT/rtl/core/pe_mul_array.v" \
    "$PROJ_ROOT/rtl/core/fp16_mul.v" \
    "$PROJ_ROOT/rtl/core/fp16_add.v" \
    "$PROJ_ROOT/rtl/core/accum_bank.v" \
    "$PROJ_ROOT/rtl/core/accum_bank_16.v" \
    "$PROJ_ROOT/rtl/core/row_accumulator_16bank.v" \
    "$PROJ_ROOT/rtl/infrastructure/scratchpad.v"

echo "[OK] Compile passed."
echo
echo "========================================"
echo "[2/2] Running Cocotb cluster test..."
echo "========================================"

vvp \
    -M "$COCOTB_LIB" \
    -m "$COCOTB_VPI_MODULE" \
    sim_build/sim_cluster.vvp

echo
echo "========================================"
echo "Done."
echo "========================================"
