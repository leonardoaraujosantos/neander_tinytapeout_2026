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
    echo "Creating build directory..."
    mkdir -p build
    echo "Building lburg first..."
    make BUILDDIR=build lburg
    echo "Generating Neander-X backend from grammar..."
    ./build/lburg src/neanderx.md > build/neanderx.c
    echo "Building LCC with Neander-X target..."
    make BUILDDIR=build TARGET=neanderx HOSTFILE=etc/neanderx.c rcc
    echo ""
    echo "LCC build complete!"
    echo "  Compiler: {{LCC_DIR}}/build/rcc"
    echo "  Usage: ./build/rcc -target=neanderx input.c > output.s"

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
    ./build/lburg src/neanderx.md > build/neanderx.c
    make BUILDDIR=build TARGET=neanderx HOSTFILE=etc/neanderx.c rcc
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
    {{LCC_DIR}}/build/rcc -target=neanderx "$INPUT" > "$OUTPUT"

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
        {{LCC_DIR}}/build/rcc -target=neanderx "$cfile" > "$sfile"
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

# Run all cocotb tests (both CPU and LCC)
test: test-cpu test-lcc

# Run cocotb tests for LCC programs
test-lcc:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Running LCC program tests..."
    cd {{COCOTB_DIR}}
    make clean
    make

# Run cocotb tests for CPU (basic tests)
test-cpu:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Running CPU tests..."
    cd cocotb_tests
    make clean
    make

# Run a specific LCC test
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
        {{LCC_DIR}}/build/rcc -target=neanderx -version 2>&1 || true
    else
        echo "LCC not found. Run 'just build-lcc' to install."
    fi

# Watch for changes and recompile (requires entr)
watch:
    @echo "Watching for changes in {{SAMPLES_DIR}}/*.c"
    @echo "Press Ctrl+C to stop"
    ls {{SAMPLES_DIR}}/*.c | entr -c just compile

# ============================================================================
# GDS Hardening (TinyTapeout / LibreLane for IHP SG13G2)
# ============================================================================

# Configuration for IHP hardening
TT_TOOLS_REPO := "https://github.com/TinyTapeout/tt-support-tools.git"
TT_TOOLS_DIR := "tt"
TT_VENV_DIR := ".venv_tt"
LIBRELANE_VERSION := "3.0.0.dev44"

# Setup TinyTapeout tools and environment
setup-tt:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Setting up TinyTapeout tools..."

    # Check for system dependencies on macOS
    if [[ "$(uname)" == "Darwin" ]]; then
        if ! brew list cairo &>/dev/null; then
            echo "Installing cairo via Homebrew (required for PNG rendering)..."
            brew install cairo
        fi
        if ! brew list pkg-config &>/dev/null; then
            echo "Installing pkg-config via Homebrew..."
            brew install pkg-config
        fi
    fi

    # Clone tt-support-tools if not present
    if [ ! -d "{{TT_TOOLS_DIR}}" ]; then
        echo "Cloning tt-support-tools..."
        git clone {{TT_TOOLS_REPO}} {{TT_TOOLS_DIR}}
    else
        echo "tt-support-tools already present, updating..."
        cd {{TT_TOOLS_DIR}} && git pull && cd ..
    fi

    # Create virtual environment if not present
    if [ ! -d "{{TT_VENV_DIR}}" ]; then
        echo "Creating Python virtual environment..."
        python3 -m venv {{TT_VENV_DIR}}
    fi

    # Activate and install dependencies
    echo "Installing dependencies..."
    source {{TT_VENV_DIR}}/bin/activate

    # On macOS, set up paths for Homebrew libraries before installing Python packages
    if [[ "$(uname)" == "Darwin" ]]; then
        BREW_PREFIX="$(brew --prefix)"
        export PKG_CONFIG_PATH="${BREW_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
        export LDFLAGS="-L${BREW_PREFIX}/lib"
        export CPPFLAGS="-I${BREW_PREFIX}/include"

        # Create sitecustomize.py to help cffi find Homebrew libraries
        SITE_PACKAGES=$(python3 -c "import site; print(site.getsitepackages()[0])")
        echo "Creating sitecustomize.py for cairo library path..."
        printf '%s\n' \
            'import os' \
            'import ctypes' \
            '' \
            '# Help cffi/cairocffi find Homebrew libraries on macOS' \
            'brew_lib = "/opt/homebrew/lib"' \
            'if os.path.exists(brew_lib):' \
            '    try:' \
            '        ctypes.CDLL(os.path.join(brew_lib, "libcairo.2.dylib"))' \
            '    except OSError:' \
            '        pass' \
            > "${SITE_PACKAGES}/sitecustomize.py"
    fi

    pip install --upgrade pip
    pip install -r {{TT_TOOLS_DIR}}/requirements.txt
    pip install librelane=={{LIBRELANE_VERSION}}

    echo ""
    echo "Setup complete!"
    echo "  tt-support-tools: {{TT_TOOLS_DIR}}/"
    echo "  Virtual env: {{TT_VENV_DIR}}/"
    echo "  LibreLane: {{LIBRELANE_VERSION}}"

# Run GDS hardening with LibreLane (IHP SG13G2)
gds_harden:
    #!/usr/bin/env bash
    set -euo pipefail

    # Check if setup was done
    if [ ! -d "{{TT_TOOLS_DIR}}" ] || [ ! -d "{{TT_VENV_DIR}}" ]; then
        echo "TinyTapeout tools not set up. Running setup first..."
        just setup-tt
    fi

    echo "Running GDS hardening for IHP SG13G2..."
    echo "  LibreLane: {{LIBRELANE_VERSION}}"
    echo ""

    source {{TT_VENV_DIR}}/bin/activate

    # Set environment for IHP
    export PDK=ihp-sg13g2

    # On macOS, create a wrapper to handle library paths (SIP strips DYLD_* vars)
    if [[ "$(uname)" == "Darwin" ]]; then
        BREW_PREFIX="$(brew --prefix)"
        CAIRO_LIB="${BREW_PREFIX}/lib/libcairo.2.dylib"

        # Create symlinks in the venv lib if they don't exist
        VENV_LIB="{{TT_VENV_DIR}}/lib"
        mkdir -p "$VENV_LIB"
        if [ ! -e "$VENV_LIB/libcairo.2.dylib" ]; then
            echo "Creating cairo library symlink in venv..."
            ln -sf "$CAIRO_LIB" "$VENV_LIB/libcairo.2.dylib"
            ln -sf "${BREW_PREFIX}/lib/libcairo.dylib" "$VENV_LIB/libcairo.dylib"
        fi

        # Set library path to include venv lib
        export DYLD_FALLBACK_LIBRARY_PATH="${VENV_LIB}:${BREW_PREFIX}/lib:${DYLD_FALLBACK_LIBRARY_PATH:-}"
    fi

    # Create user config
    echo "Creating user config..."
    ./{{TT_TOOLS_DIR}}/tt_tool.py --create-user-config --ihp

    # Run hardening
    echo ""
    echo "Running LibreLane hardening (this may take several minutes)..."
    ./{{TT_TOOLS_DIR}}/tt_tool.py --harden --ihp

    echo ""
    echo "Checking for warnings..."
    ./{{TT_TOOLS_DIR}}/tt_tool.py --print-warnings --ihp || true

    echo ""
    echo "GDS hardening complete!"
    echo "Output: runs/wokwi/"

# Helper to set up environment for tt_tool.py
[private]
tt-env:
    #!/usr/bin/env bash
    # This is sourced by other recipes

# View GDS in OpenROAD GUI
view_gds:
    #!/usr/bin/env bash
    set -euo pipefail

    if [ ! -d "{{TT_VENV_DIR}}" ]; then
        echo "Run 'just setup-tt' first"
        exit 1
    fi

    source {{TT_VENV_DIR}}/bin/activate
    [[ "$(uname)" == "Darwin" ]] && export DYLD_FALLBACK_LIBRARY_PATH="$(brew --prefix)/lib:${DYLD_FALLBACK_LIBRARY_PATH:-}"
    export PDK=ihp-sg13g2

    echo "Opening GDS in OpenROAD GUI..."
    ./{{TT_TOOLS_DIR}}/tt_tool.py --open-in-openroad --ihp

# View GDS in KLayout
view_klayout:
    #!/usr/bin/env bash
    set -euo pipefail

    if [ ! -d "{{TT_VENV_DIR}}" ]; then
        echo "Run 'just setup-tt' first"
        exit 1
    fi

    source {{TT_VENV_DIR}}/bin/activate
    [[ "$(uname)" == "Darwin" ]] && export DYLD_FALLBACK_LIBRARY_PATH="$(brew --prefix)/lib:${DYLD_FALLBACK_LIBRARY_PATH:-}"
    export PDK=ihp-sg13g2

    echo "Opening GDS in KLayout..."
    ./{{TT_TOOLS_DIR}}/tt_tool.py --open-in-klayout --ihp

# Generate PNG preview of the GDS layout
gds_png:
    #!/usr/bin/env bash
    set -euo pipefail

    if [ ! -d "{{TT_VENV_DIR}}" ]; then
        echo "Run 'just setup-tt' first"
        exit 1
    fi

    source {{TT_VENV_DIR}}/bin/activate
    [[ "$(uname)" == "Darwin" ]] && export DYLD_FALLBACK_LIBRARY_PATH="$(brew --prefix)/lib:${DYLD_FALLBACK_LIBRARY_PATH:-}"
    export PDK=ihp-sg13g2

    echo "Generating PNG preview..."
    ./{{TT_TOOLS_DIR}}/tt_tool.py --create-png --ihp
    echo "PNG generation complete!"

# Print hardening warnings
gds_warnings:
    #!/usr/bin/env bash
    set -euo pipefail

    if [ ! -d "{{TT_VENV_DIR}}" ]; then
        echo "Run 'just setup-tt' first"
        exit 1
    fi

    source {{TT_VENV_DIR}}/bin/activate
    [[ "$(uname)" == "Darwin" ]] && export DYLD_FALLBACK_LIBRARY_PATH="$(brew --prefix)/lib:${DYLD_FALLBACK_LIBRARY_PATH:-}"
    export PDK=ihp-sg13g2

    ./{{TT_TOOLS_DIR}}/tt_tool.py --print-warnings --ihp

# Clean GDS build artifacts
clean-gds:
    #!/usr/bin/env bash
    echo "Cleaning GDS artifacts..."
    rm -rf runs/
    rm -f src/user_config.json
    rm -f src/config_merged.json
    echo "GDS clean complete"

# Clean TinyTapeout setup (keeps tt-support-tools)
clean-tt:
    #!/usr/bin/env bash
    echo "Cleaning TinyTapeout virtual environment..."
    rm -rf {{TT_VENV_DIR}}
    echo "Clean complete (tt-support-tools kept)"

# Full clean including tt-support-tools
clean-tt-all:
    #!/usr/bin/env bash
    echo "Cleaning all TinyTapeout files..."
    rm -rf {{TT_VENV_DIR}}
    rm -rf {{TT_TOOLS_DIR}}
    rm -rf runs/
    rm -f src/user_config.json
    rm -f src/config_merged.json
    echo "Full TinyTapeout clean complete"
