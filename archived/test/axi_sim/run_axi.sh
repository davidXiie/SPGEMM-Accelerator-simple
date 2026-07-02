#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
PROJ_ROOT="${PROJ_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"

export COCOTB_TEST_MODULES="${COCOTB_TEST_MODULES:-test_accelerator_axi}"
export COCOTB_TESTCASE="${COCOTB_TESTCASE:-test_axi_case1}"
export COCOTB_TOPLEVEL="${COCOTB_TOPLEVEL:-tb_accelerator_axi}"
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
echo "[1/2] Compiling AXI accelerator..."
echo "========================================"

mkdir -p sim_build

iverilog -g2012 \
    -DCOCOTB_SIM=1 \
    -DC_ROW_ADDR_BITS="${C_ROW_ADDR_BITS:-8}" \
    -I"$PROJ_ROOT/rtl/include" \
    -s "$COCOTB_TOPLEVEL" \
    -o sim_build/sim_axi.vvp \
    "$PROJ_ROOT/rtl/sim/tb_accelerator_axi.v" \
    "$PROJ_ROOT/rtl/core/accelerator_axi_top.v" \
    "$PROJ_ROOT/rtl/core/ddr_model.v" \
    "$PROJ_ROOT/rtl/core/axi_loader.v" \
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
echo "[2/2] Running AXI accelerator test..."
echo "COCOTB_TESTCASE=$COCOTB_TESTCASE"
echo

vvp -M "$COCOTB_LIB" -m "$COCOTB_VPI_MODULE" sim_build/sim_axi.vvp
echo "Done."
