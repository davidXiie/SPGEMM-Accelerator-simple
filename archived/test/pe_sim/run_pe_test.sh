#!/usr/bin/env bash
# Run the PE cocotb test on Ubuntu/Linux.
# Usage:
#   conda activate gcnenv        # or activate any Python environment containing cocotb
#   chmod +x run_pe_test.sh
#   ./run_pe_test.sh
#
# Optional overrides:
#   PROJ_ROOT=/path/to/SPGEMM-Accelerator-simple ./run_pe_test.sh
#   COCOTB_LIB=/path/to/site-packages/cocotb/libs ./run_pe_test.sh

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# The original batch file is located at: <project-root>/test/pe_sim/run_pe_test.bat
PROJ_ROOT="${PROJ_ROOT:-$(cd -- "$SCRIPT_DIR/../.." && pwd)}"

# Run from the test directory so relative paths and generated files match the .bat behavior.
cd "$SCRIPT_DIR"

for cmd in python3 iverilog vvp; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "[FAIL] Required command not found: $cmd" >&2
        echo "       Please install Icarus Verilog and activate a Python/Conda environment with cocotb." >&2
        exit 1
    fi
done

# The active Python environment must contain cocotb.  Resolve its Linux library path
# automatically instead of relying on the Windows site-packages path from the .bat file.
if ! python3 -c 'import cocotb' >/dev/null 2>&1; then
    echo "[FAIL] cocotb is not available in the active Python environment." >&2
    echo "       Example: conda activate gcnenv" >&2
    exit 1
fi

COCOTB_LIB="${COCOTB_LIB:-$(python3 -c 'import cocotb; from pathlib import Path; print(Path(cocotb.__file__).resolve().parent / "libs")')}"
if [[ ! -d "$COCOTB_LIB" ]]; then
    echo "[FAIL] Cocotb shared-library directory does not exist: $COCOTB_LIB" >&2
    exit 1
fi

export MODULE="${MODULE:-test_pe}"
export TESTCASE="${TESTCASE:-test_pe_50x50}"
export TOPLEVEL="${TOPLEVEL:-tb_pe_top}"
export COCOTB_LOG_LEVEL="${COCOTB_LOG_LEVEL:-INFO}"

# Some cocotb setups use this variable internally; exporting it also makes the resolved
# path visible to any child process invoked by the test.
export COCOTB_LIB

echo "========================================"
echo "[1/2] Compiling PE testbench..."
echo "========================================"
mkdir -p sim_build

if ! iverilog -g2012 -DCOCOTB_SIM=1 -I"$PROJ_ROOT/rtl/include" \
    -s tb_pe_top \
    -o sim_build/sim.vvp \
    "$PROJ_ROOT/rtl/sim/tb_pe_top.v" \
    "$PROJ_ROOT/rtl/core/pe_top.v" \
    "$PROJ_ROOT/rtl/core/pe_task_packer.v" \
    "$PROJ_ROOT/rtl/core/pe_serializer.v" \
    "$PROJ_ROOT/rtl/core/pe_accumulator.v" \
    "$PROJ_ROOT/rtl/core/pe_mul_array.v" \
    "$PROJ_ROOT/rtl/infrastructure/scratchpad.v"; then
    echo "[FAIL] Compile error" >&2
    exit 1
fi

echo "[OK] Compile passed."
echo
echo "========================================"
echo "[2/2] Running cocotb test..."
echo "========================================"

if ! vvp -M "$COCOTB_LIB" -m cocotbvpi_icarus sim_build/sim.vvp; then
    echo "[FAIL] Cocotb test failed" >&2
    exit 1
fi

echo
echo "========================================"
echo "Done."
echo "========================================"
