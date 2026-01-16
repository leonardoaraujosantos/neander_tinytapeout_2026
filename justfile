# Justfile for Neander-X CPU development
# =======================================
#
# Usage:
#   just              - Show available recipes
#   just build-lcc    - Build the LCC compiler
#   just compile      - Compile all C samples to assembly
#   just assemble     - Assemble all .s files to binary
#   just test         - Run cocotb tests
#   just all          - Build, compile, assemble, and test

# Configuration
LCC_REPO := "https://github.com/leonardoaraujosantos/lcc-neanderx.git"
LCC_DIR := "../lcc-neanderx"
SAMPLES_DIR := "lcc_samples"
ASSEMBLY_DIR := "neander_assembly"
COCOTB_DIR := "cocotb_tests_lcc"

# Default recipe - show help
default:
    @just --list

# Build the LCC compiler from source
build-lcc:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ ! -d "{{LCC_DIR}}" ]; then
        echo "Cloning LCC repository..."
        git clone {{LCC_REPO}} {{LCC_DIR}}
    fi
    cd {{LCC_DIR}}
    echo "Building lburg..."
    cd lburg && make && cd ..
    echo "Generating Neander-X backend..."
    ./lburg/lburg src/neanderx.md > src/neanderx.c
    echo "Building LCC..."
    make
    echo "LCC build complete: {{LCC_DIR}}/build/rcc"

# Update LCC from git repository
update-lcc:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ ! -d "{{LCC_DIR}}" ]; then
        echo "LCC not found. Run 'just build-lcc' first."
        exit 1
    fi
    cd {{LCC_DIR}}
    git pull
    echo "Rebuilding LCC..."
    ./lburg/lburg src/neanderx.md > src/neanderx.c
    make
    echo "LCC update complete"

# Compile a single C file to assembly
compile-one FILE:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ ! -f "{{LCC_DIR}}/build/rcc" ]; then
        echo "LCC not found. Run 'just build-lcc' first."
        exit 1
    fi
    INPUT="{{FILE}}"
    OUTPUT="${INPUT%.c}.s"
    echo "Compiling $INPUT -> $OUTPUT"
    {{LCC_DIR}}/build/rcc -target=neanderx/none "$INPUT" > "$OUTPUT"

# Compile all C samples to assembly
compile:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ ! -f "{{LCC_DIR}}/build/rcc" ]; then
        echo "LCC not found. Run 'just build-lcc' first."
        exit 1
    fi
    echo "Compiling all C samples..."
    for cfile in {{SAMPLES_DIR}}/*.c; do
        sfile="${cfile%.c}.s"
        echo "  $cfile -> $sfile"
        {{LCC_DIR}}/build/rcc -target=neanderx/none "$cfile" > "$sfile"
    done
    echo "Compilation complete"

# Assemble a single .s file to binary
assemble-one FILE:
    #!/usr/bin/env bash
    set -euo pipefail
    INPUT="{{FILE}}"
    BINFILE="${INPUT%.s}.bin"
    HEXFILE="${INPUT%.s}.hex"
    echo "Assembling $INPUT"
    python3 {{ASSEMBLY_DIR}}/neanderasm.py "$INPUT" -o "$BINFILE"
    python3 {{ASSEMBLY_DIR}}/neanderasm.py "$INPUT" -o "$HEXFILE" --format hex
    echo "  -> $BINFILE"
    echo "  -> $HEXFILE"

# Assemble all .s files in lcc_samples to binary
assemble:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Assembling all samples..."
    for sfile in {{SAMPLES_DIR}}/*.s; do
        BINFILE="${sfile%.s}.bin"
        HEXFILE="${sfile%.s}.hex"
        echo "  $sfile -> .bin, .hex"
        python3 {{ASSEMBLY_DIR}}/neanderasm.py "$sfile" -o "$BINFILE" 2>/dev/null || echo "    (skipped - assembly error)"
        python3 {{ASSEMBLY_DIR}}/neanderasm.py "$sfile" -o "$HEXFILE" --format hex 2>/dev/null || true
    done
    echo "Assembly complete"

# Generate Python byte arrays for cocotb testing
generate-python:
    python3 scripts/generate_lcc_programs.py

# Run cocotb tests for LCC programs
test:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Running LCC program tests..."
    cd {{COCOTB_DIR}}
    make clean
    make

# Run a specific test
test-one NAME:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Running test: {{NAME}}"
    cd {{COCOTB_DIR}}
    make TESTCASE=test_{{NAME}}

# Run quick test (first 3 samples only)
test-quick:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Running quick tests (01-03)..."
    cd {{COCOTB_DIR}}
    make TESTCASE=test_01_hello,test_02_locals,test_03_arithmetic

# Clean build artifacts
clean:
    #!/usr/bin/env bash
    echo "Cleaning build artifacts..."
    rm -f {{SAMPLES_DIR}}/*.bin
    rm -f {{SAMPLES_DIR}}/*.hex
    rm -rf {{COCOTB_DIR}}/__pycache__
    rm -rf {{COCOTB_DIR}}/sim_build
    rm -f {{COCOTB_DIR}}/results.xml
    rm -f {{COCOTB_DIR}}/*.vcd
    echo "Clean complete"

# Clean everything including LCC
clean-all: clean
    #!/usr/bin/env bash
    echo "Removing LCC directory..."
    rm -rf {{LCC_DIR}}
    echo "Clean all complete"

# Full build: compile, assemble, and test
all: compile assemble test

# Full rebuild from scratch
rebuild: clean-all build-lcc compile assemble test

# Show sample info
info:
    @echo "LCC Samples:"
    @echo "============"
    @ls -1 {{SAMPLES_DIR}}/*.c | xargs -n1 basename | sed 's/.c$//'
    @echo ""
    @echo "Expected Results:"
    @echo "================="
    @echo "  01_hello:      42"
    @echo "  02_locals:     300"
    @echo "  03_arithmetic: 100"
    @echo "  04_globals:    15"
    @echo "  05_loop:       55"
    @echo "  06_array:      150"
    @echo "  07_factorial:  120"
    @echo "  08_fibonacci:  55"
    @echo "  09_bitwise:    8190"
    @echo "  10_char:       145"

# Check LCC installation
check-lcc:
    #!/usr/bin/env bash
    if [ -f "{{LCC_DIR}}/build/rcc" ]; then
        echo "LCC installed at: {{LCC_DIR}}/build/rcc"
        {{LCC_DIR}}/build/rcc -target=neanderx/none -version 2>&1 || true
    else
        echo "LCC not found. Run 'just build-lcc' to install."
    fi

# Watch for changes and recompile (requires entr)
watch:
    @echo "Watching for changes in {{SAMPLES_DIR}}/*.c"
    @echo "Press Ctrl+C to stop"
    ls {{SAMPLES_DIR}}/*.c | entr -c just compile
