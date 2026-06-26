#!/usr/bin/env bash
#
# Ubuntu/Linux runner for tb_pe_top + Cocotb 2.x + Icarus Verilog.
#
# Usage:
#   conda activate py311
#   chmod +x run_comp_cocotb2_fixed.sh
#   ./run_comp_cocotb2_fixed.sh
#
# Optional:
#   PROJ_ROOT=/path/to/SPGEMM-Accelerator-simple ./run_comp_cocotb2_fixed.sh
#   COCOTB_TESTCASE=test_comp_case1_p1 ./run_comp_cocotb2_fixed.sh

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Expected layout:
#   <PROJ_ROOT>/test/pe_sim/run_comp_cocotb2_fixed.sh
PROJ_ROOT="${PROJ_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"

# Cocotb 2.x test configuration.
export COCOTB_TEST_MODULES="${COCOTB_TEST_MODULES:-test_comp}"
export COCOTB_TESTCASE="${COCOTB_TESTCASE:-test_comp_case1_p0}"
export COCOTB_TOPLEVEL="${COCOTB_TOPLEVEL:-tb_pe_top}"
export COCOTB_LOG_LEVEL="${COCOTB_LOG_LEVEL:-INFO}"
export COCOTB_SIM=1
export PYTHONIOENCODING="${PYTHONIOENCODING:-utf-8}"

# Allow Python to import test_comp.py from this script's directory.
export PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"

# Check required tools.
for tool in iverilog vvp cocotb-config; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "[FAIL] '$tool' was not found in PATH." >&2
        if [[ "$tool" == "iverilog" || "$tool" == "vvp" ]]; then
            echo "       Install Icarus Verilog with:" >&2
            echo "       sudo apt update && sudo apt install -y iverilog" >&2
        else
            echo "       Activate the Conda/Python environment containing Cocotb." >&2
            echo "       Example: conda activate py311" >&2
        fi
        exit 1
    fi
done

# Get Cocotb and Python paths from the active Conda/Python environment.
COCOTB_LIB="$(cocotb-config --lib-dir)"
COCOTB_VPI_MODULE="$(cocotb-config --lib-name vpi icarus)"
LIBPYTHON_LOC="$(cocotb-config --libpython)"
PYGPI_PYTHON_BIN="$(cocotb-config --python-bin)"

if [[ ! -d "$COCOTB_LIB" ]]; then
    echo "[FAIL] Cocotb library directory does not exist:" >&2
    echo "       $COCOTB_LIB" >&2
    exit 1
fi

if [[ ! -f "$LIBPYTHON_LOC" ]]; then
    echo "[FAIL] Python shared library does not exist:" >&2
    echo "       $LIBPYTHON_LOC" >&2
    echo "       Reinstall Python in this Conda environment if needed." >&2
    exit 1
fi

export LIBPYTHON_LOC
export PYGPI_PYTHON_BIN

# Make Conda's libpython visible to vvp at runtime.
export LD_LIBRARY_PATH="$(dirname "$LIBPYTHON_LOC")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

echo "========================================"
echo "[1/2] Compiling PE testbench..."
echo "========================================"

mkdir -p sim_build

iverilog -g2012 \
    -DCOCOTB_SIM=1 \
    -I"$PROJ_ROOT/rtl/include" \
    -s "$COCOTB_TOPLEVEL" \
    -o sim_build/sim.vvp \
    "$PROJ_ROOT/rtl/sim/tb_pe_top.v" \
    "$PROJ_ROOT/rtl/core/pe_top.v" \
    "$PROJ_ROOT/rtl/core/pe_mul_array.v" \
    "$PROJ_ROOT/rtl/core/fp16_mul.v" \
    "$PROJ_ROOT/rtl/core/fp16_add.v" \
    "$PROJ_ROOT/rtl/core/accum_bank.v" \
    "$PROJ_ROOT/rtl/core/row_accumulator_4bank.v" \
    "$PROJ_ROOT/rtl/infrastructure/scratchpad.v"

echo "[OK] Compile passed."
echo
echo "========================================"
echo "[2/2] Running Cocotb test..."
echo "========================================"
echo "COCOTB_TEST_MODULES=$COCOTB_TEST_MODULES"
echo "COCOTB_TESTCASE=$COCOTB_TESTCASE"
echo "COCOTB_TOPLEVEL=$COCOTB_TOPLEVEL"
echo "PYGPI_PYTHON_BIN=$PYGPI_PYTHON_BIN"
echo "LIBPYTHON_LOC=$LIBPYTHON_LOC"
echo

vvp \
    -M "$COCOTB_LIB" \
    -m "$COCOTB_VPI_MODULE" \
    sim_build/sim.vvp

echo
echo "========================================"
echo "Done."
echo "========================================"
