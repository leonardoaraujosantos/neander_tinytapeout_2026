#!/usr/bin/env python3
"""
Neander-X 16-bit Assembler
==========================

Assembler for the Neander-X 16-bit CPU, generating binary output for cocotb simulation.

Usage:
    python neanderasm.py input.s -o output.bin
    python neanderasm.py input.s -o output.hex --format hex
    python neanderasm.py input.s -o output.mem --format mem
"""

import argparse
import re
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Union


# =============================================================================
# Instruction Set Definition
# =============================================================================

# Single-byte instructions (no operand)
SINGLE_BYTE_OPCODES = {
    'NOP':     0x00,
    'NEG':     0x01,
    'TAY':     0x03,
    'TYA':     0x04,
    'INY':     0x05,
    'MUL':     0x09,
    'TSF':     0x0A,
    'TFS':     0x0B,
    'PUSH_FP': 0x0C,
    'POP_FP':  0x0D,
    'DIV':     0x0E,
    'MOD':     0x0F,
    'DEX':     0x18,
    'DEY':     0x19,
    'TAB':     0x1C,   # Transfer AC to B
    'TBA':     0x1D,   # Transfer B to AC
    'ADDX':    0x32,
    'SUBX':    0x33,
    'ADDY':    0x34,
    'SUBY':    0x35,
    'INB':     0x36,   # Increment B
    'DEB':     0x37,   # Decrement B
    'SWPB':    0x38,   # Swap AC <-> B
    'ADDB':    0x39,   # AC = AC + B
    'SUBB':    0x3A,   # AC = AC - B
    'NOT':     0x60,
    'ASR':     0x61,
    'PUSH':    0x70,
    'POP':     0x71,
    'RET':     0x73,
    'INC':     0x75,
    'DEC':     0x76,
    'SHL':     0x78,
    'SHR':     0x79,
    'TAX':     0x7D,
    'TXA':     0x7E,
    'INX':     0x7F,
    'HLT':     0xF0,
}

# Three-byte instructions (16-bit immediate operand)
# Note: The CPU uses 16-bit immediate values for these instructions
IMM16_OPCODES = {
    'LDI':  0xE0,
    'LDXI': 0x7C,
    'LDYI': 0x06,
    'LDBI': 0xE4,   # Load B immediate
}

# Three-byte instructions (16-bit address operand)
ADDR16_OPCODES = {
    'CMP':  0x02,
    'LDY':  0x07,
    'STY':  0x08,
    'STA':  0x10,
    'LDB':  0x1E,   # Load B from memory
    'STB':  0x1F,   # Store B to memory
    'LDA':  0x20,
    'ADD':  0x30,
    'ADC':  0x31,
    'OR':   0x40,
    'AND':  0x50,
    'SBC':  0x51,
    'SUB':  0x74,
    'XOR':  0x77,
    'LDX':  0x7A,
    'STX':  0x7B,
    'JMP':  0x80,
    'JC':   0x81,
    'JNC':  0x82,
    'JLE':  0x83,
    'JGT':  0x84,
    'JGE':  0x85,
    'JBE':  0x86,
    'JA':   0x87,
    'PUSH_ADDR': 0x89,  # Push MEM[addr] to stack
    'POP_ADDR':  0x8A,  # Pop from stack to MEM[addr]
    'JN':   0x90,
    'JZ':   0xA0,
    'JNZ':  0xB0,
    'IN':   0xC0,
    'OUT':  0xD0,
    'CALL': 0x72,
}

# Indexed addressing variants (base opcode + offset)
INDEXED_OPCODES = {
    # LDA variants
    'LDA,X':  0x21,  # LDA addr,X
    'LDA,Y':  0x22,  # LDA addr,Y
    'LDA,FP': 0x24,  # LDA addr,FP (frame pointer relative)
    'LDA,SP': 0x28,  # LDA off,SP (stack pointer relative)
    # STA variants
    'STA,X':  0x11,  # STA addr,X
    'STA,Y':  0x12,  # STA addr,Y
    'STA,FP': 0x14,  # STA addr,FP
    'STA,SP': 0x17,  # STA off,SP
}

# Instructions that support implicit FP-relative addressing when operand is a number
FP_RELATIVE_INSTRUCTIONS = {'LDA', 'STA'}


# =============================================================================
# Data Structures
# =============================================================================

@dataclass
class Instruction:
    """Represents a parsed instruction."""
    mnemonic: str
    operand: Optional[str]
    index_reg: Optional[str]  # X, Y, FP, SP
    line_num: int
    address: int = 0
    size: int = 0


@dataclass
class DataDirective:
    """Represents a data directive (.byte, .word)."""
    directive: str
    values: List[str]
    line_num: int
    address: int = 0
    size: int = 0


@dataclass
class Label:
    """Represents a label definition."""
    name: str
    address: int
    line_num: int


# =============================================================================
# Expression Evaluator
# =============================================================================

class ExpressionEvaluator:
    """Evaluates assembly expressions with lo(), hi(), and arithmetic."""

    def __init__(self, symbols: Dict[str, int]):
        self.symbols = symbols

    def evaluate(self, expr: str, current_addr: int = 0) -> int:
        """Evaluate an expression and return its value."""
        expr = expr.strip()

        # Handle lo() and hi() functions
        lo_match = re.match(r'^lo\s*\(\s*(.+)\s*\)$', expr, re.IGNORECASE)
        if lo_match:
            inner = self.evaluate(lo_match.group(1), current_addr)
            return inner & 0xFF

        hi_match = re.match(r'^hi\s*\(\s*(.+)\s*\)$', expr, re.IGNORECASE)
        if hi_match:
            inner = self.evaluate(hi_match.group(1), current_addr)
            return (inner >> 8) & 0xFF

        # Handle special $ for current address
        if expr == '$':
            return current_addr

        # Handle addition/subtraction with proper precedence
        # Look for + or - not inside parentheses
        depth = 0
        for i in range(len(expr) - 1, -1, -1):
            c = expr[i]
            if c == ')':
                depth += 1
            elif c == '(':
                depth -= 1
            elif depth == 0 and c in '+-':
                left = expr[:i].strip()
                right = expr[i+1:].strip()
                if left:  # Not a unary operator
                    left_val = self.evaluate(left, current_addr)
                    right_val = self.evaluate(right, current_addr)
                    if c == '+':
                        return (left_val + right_val) & 0xFFFF
                    else:
                        return (left_val - right_val) & 0xFFFF

        # Handle parentheses
        if expr.startswith('(') and expr.endswith(')'):
            return self.evaluate(expr[1:-1], current_addr)

        # Handle hex numbers
        if expr.startswith('0x') or expr.startswith('0X'):
            return int(expr, 16)
        if expr.startswith('$'):
            return int(expr[1:], 16)

        # Handle binary numbers
        if expr.startswith('0b') or expr.startswith('0B'):
            return int(expr, 2)
        if expr.startswith('%'):
            return int(expr[1:], 2)

        # Handle decimal numbers
        try:
            return int(expr)
        except ValueError:
            pass

        # Handle negative numbers
        if expr.startswith('-'):
            return (-self.evaluate(expr[1:], current_addr)) & 0xFFFF

        # Handle character literals
        if expr.startswith("'") and expr.endswith("'") and len(expr) == 3:
            return ord(expr[1])

        # Look up symbol
        if expr in self.symbols:
            return self.symbols[expr]

        raise ValueError(f"Unknown symbol or invalid expression: {expr}")


# =============================================================================
# Assembler
# =============================================================================

class NeanderAssembler:
    """Two-pass assembler for Neander-X CPU."""

    def __init__(self):
        self.symbols: Dict[str, int] = {}
        self.memory: Dict[int, int] = {}
        self.current_address: int = 0
        self.errors: List[str] = []
        self.warnings: List[str] = []
        self.elements: List[Union[Instruction, DataDirective, Label]] = []
        self.origin: int = 0
        self.max_address: int = 0

    def error(self, line_num: int, msg: str):
        """Record an error."""
        self.errors.append(f"Line {line_num}: {msg}")

    def warning(self, line_num: int, msg: str):
        """Record a warning."""
        self.warnings.append(f"Line {line_num}: {msg}")

    def parse_line(self, line: str, line_num: int) -> Optional[Union[Instruction, DataDirective, Label, str]]:
        """Parse a single assembly line."""
        # Remove comments
        comment_idx = line.find(';')
        if comment_idx != -1:
            line = line[:comment_idx]

        line = line.strip()
        if not line:
            return None

        # Check for label definition
        label_match = re.match(r'^([A-Za-z_][A-Za-z0-9_]*):(.*)$', line)
        if label_match:
            label_name = label_match.group(1)
            rest = label_match.group(2).strip()
            # Return label and recursively parse the rest
            return ('label', label_name, rest)

        # Check for directives
        if line.startswith('.'):
            return self.parse_directive(line, line_num)

        # Parse instruction
        return self.parse_instruction(line, line_num)

    def parse_directive(self, line: str, line_num: int) -> Optional[Union[DataDirective, str]]:
        """Parse an assembler directive."""
        parts = line.split(None, 1)
        directive = parts[0].lower()
        args = parts[1] if len(parts) > 1 else ""

        if directive == '.org':
            return ('org', args.strip())
        elif directive == '.byte':
            values = [v.strip() for v in args.split(',')]
            return DataDirective('.byte', values, line_num)
        elif directive == '.word':
            values = [v.strip() for v in args.split(',')]
            return DataDirective('.word', values, line_num)
        elif directive == '.global':
            return ('global', args.strip())
        elif directive == '.text':
            return None  # Ignore .text directive
        elif directive == '.data':
            return None  # Ignore .data directive
        elif directive == '.bss':
            return None  # Ignore .bss directive
        elif directive == '.space':
            # .space n - reserve n bytes (initialize to 0)
            return ('space', args.strip())
        elif directive == '.align':
            # .align n - align to n-byte boundary
            return ('align', args.strip())
        else:
            self.warning(line_num, f"Unknown directive: {directive}")
            return None

    def parse_instruction(self, line: str, line_num: int) -> Optional[Instruction]:
        """Parse an instruction."""
        # Split into mnemonic and operand
        parts = line.split(None, 1)
        mnemonic = parts[0].upper()
        operand = parts[1].strip() if len(parts) > 1 else None

        # Check for indexed addressing (operand contains comma with index register)
        index_reg = None
        if operand:
            # Match patterns like "addr,X" or "addr,FP" or "offset+1"
            idx_match = re.match(r'^(.+),\s*(X|Y|FP|SP)$', operand, re.IGNORECASE)
            if idx_match:
                operand = idx_match.group(1).strip()
                index_reg = idx_match.group(2).upper()

        return Instruction(mnemonic, operand, index_reg, line_num)

    def is_numeric_expression(self, operand: str) -> bool:
        """Check if operand is a pure numeric expression (for implicit FP-relative).

        Returns True if the operand is:
        - A number (decimal, hex, binary)
        - An expression with numbers and arithmetic (e.g., -2+1, 4+1)
        - Does NOT contain symbol references (labels starting with letter/underscore)

        Returns False if the operand:
        - Contains symbol references (e.g., _tmp, label)
        - Uses lo() or hi() functions (which typically reference symbols)
        """
        if not operand:
            return False

        # Check for lo/hi functions - these typically reference symbols
        if re.search(r'\b(lo|hi)\s*\(', operand, re.IGNORECASE):
            return False

        # Check if it's purely numeric/arithmetic
        # Allow: digits, hex prefix, binary prefix, +, -, parentheses, spaces
        # Disallow: letters that look like identifiers (but allow hex digits a-f)

        # Remove all valid numeric components
        test = operand
        # Remove hex numbers like 0x1234
        test = re.sub(r'0[xX][0-9a-fA-F]+', '', test)
        # Remove binary numbers like 0b1010
        test = re.sub(r'0[bB][01]+', '', test)
        # Remove decimal numbers
        test = re.sub(r'\d+', '', test)
        # Remove operators and whitespace
        test = re.sub(r'[\s+\-\*\/\(\)]', '', test)

        # If anything remains, it's a symbol reference
        return len(test) == 0

    def get_instruction_size(self, instr: Instruction) -> int:
        """Calculate the size of an instruction in bytes."""
        mnemonic = instr.mnemonic

        # Check for indexed variants (explicit ,FP, ,X, etc.)
        if instr.index_reg:
            key = f"{mnemonic},{instr.index_reg}"
            if key in INDEXED_OPCODES:
                return 3  # opcode + 16-bit offset

        # Check for implicit FP-relative addressing (LDA/STA with numeric operand)
        if mnemonic in FP_RELATIVE_INSTRUCTIONS and instr.operand:
            if self.is_numeric_expression(instr.operand):
                return 3  # FP-relative addressing

        # Single-byte instructions
        if mnemonic in SINGLE_BYTE_OPCODES:
            return 1

        # Three-byte instructions (16-bit immediate)
        if mnemonic in IMM16_OPCODES:
            return 3

        # Three-byte instructions (address)
        if mnemonic in ADDR16_OPCODES:
            return 3

        return 0  # Unknown instruction

    def pass1(self, source: str):
        """First pass: collect labels and calculate addresses."""
        self.current_address = 0
        lines = source.split('\n')

        for line_num, line in enumerate(lines, 1):
            result = self.parse_line(line, line_num)

            if result is None:
                continue

            # Handle label
            if isinstance(result, tuple) and result[0] == 'label':
                label_name = result[1]
                rest = result[2]

                if label_name in self.symbols:
                    self.warning(line_num, f"Redefinition of label: {label_name}")
                self.symbols[label_name] = self.current_address

                # Process rest of line after label
                if rest:
                    result = self.parse_line(rest, line_num)
                    if result is None:
                        continue
                else:
                    continue

            # Handle org directive
            if isinstance(result, tuple) and result[0] == 'org':
                evaluator = ExpressionEvaluator(self.symbols)
                try:
                    self.current_address = evaluator.evaluate(result[1], self.current_address)
                except ValueError as e:
                    self.error(line_num, str(e))
                continue

            # Handle global directive (just skip in pass1)
            if isinstance(result, tuple) and result[0] == 'global':
                continue

            # Handle space directive (reserve bytes)
            if isinstance(result, tuple) and result[0] == 'space':
                evaluator = ExpressionEvaluator(self.symbols)
                try:
                    size = evaluator.evaluate(result[1], self.current_address)
                    self.current_address += size
                except ValueError as e:
                    self.error(line_num, str(e))
                continue

            # Handle align directive
            if isinstance(result, tuple) and result[0] == 'align':
                evaluator = ExpressionEvaluator(self.symbols)
                try:
                    alignment = evaluator.evaluate(result[1], self.current_address)
                    if alignment > 0:
                        remainder = self.current_address % alignment
                        if remainder != 0:
                            self.current_address += alignment - remainder
                except ValueError as e:
                    self.error(line_num, str(e))
                continue

            # Handle data directives
            if isinstance(result, DataDirective):
                result.address = self.current_address
                if result.directive == '.byte':
                    result.size = len(result.values)
                elif result.directive == '.word':
                    result.size = len(result.values) * 2
                self.elements.append(result)
                self.current_address += result.size
                continue

            # Handle instruction
            if isinstance(result, Instruction):
                result.address = self.current_address
                result.size = self.get_instruction_size(result)
                if result.size == 0:
                    self.error(line_num, f"Unknown instruction: {result.mnemonic}")
                else:
                    self.elements.append(result)
                    self.current_address += result.size

    def pass2(self):
        """Second pass: generate machine code."""
        evaluator = ExpressionEvaluator(self.symbols)

        for element in self.elements:
            if isinstance(element, DataDirective):
                addr = element.address
                for val_str in element.values:
                    try:
                        val = evaluator.evaluate(val_str, addr)
                    except ValueError as e:
                        self.error(element.line_num, str(e))
                        val = 0

                    if element.directive == '.byte':
                        self.memory[addr] = val & 0xFF
                        addr += 1
                    elif element.directive == '.word':
                        self.memory[addr] = val & 0xFF
                        self.memory[addr + 1] = (val >> 8) & 0xFF
                        addr += 2

                self.max_address = max(self.max_address, addr)

            elif isinstance(element, Instruction):
                addr = element.address
                mnemonic = element.mnemonic

                # Determine opcode
                opcode = None
                use_fp_relative = False

                # Check indexed variant first (explicit ,FP, ,X, etc.)
                if element.index_reg:
                    key = f"{mnemonic},{element.index_reg}"
                    if key in INDEXED_OPCODES:
                        opcode = INDEXED_OPCODES[key]

                # Check for implicit FP-relative addressing (LDA/STA with numeric operand)
                if opcode is None and mnemonic in FP_RELATIVE_INSTRUCTIONS and element.operand:
                    if self.is_numeric_expression(element.operand):
                        key = f"{mnemonic},FP"
                        if key in INDEXED_OPCODES:
                            opcode = INDEXED_OPCODES[key]
                            use_fp_relative = True

                if opcode is None:
                    if mnemonic in SINGLE_BYTE_OPCODES:
                        opcode = SINGLE_BYTE_OPCODES[mnemonic]
                    elif mnemonic in IMM16_OPCODES:
                        opcode = IMM16_OPCODES[mnemonic]
                    elif mnemonic in ADDR16_OPCODES:
                        opcode = ADDR16_OPCODES[mnemonic]

                if opcode is None:
                    self.error(element.line_num, f"Cannot encode instruction: {mnemonic}")
                    continue

                # Write opcode
                self.memory[addr] = opcode
                addr += 1

                # Write operand if needed
                if element.size >= 2 and element.operand:
                    try:
                        operand_val = evaluator.evaluate(element.operand, element.address)
                    except ValueError as e:
                        self.error(element.line_num, str(e))
                        operand_val = 0

                    if element.size == 2:
                        # 8-bit immediate
                        self.memory[addr] = operand_val & 0xFF
                        addr += 1
                    elif element.size == 3:
                        # 16-bit address (little-endian)
                        self.memory[addr] = operand_val & 0xFF
                        self.memory[addr + 1] = (operand_val >> 8) & 0xFF
                        addr += 2

                self.max_address = max(self.max_address, addr)

    def peephole_optimize(self):
        """Apply peephole optimizations to the instruction list.

        This fixes patterns generated by LCC that don't work correctly on
        the Neander-X accumulator architecture. Specifically:

        Pattern: LDA a, LDA b, STA _tmp, POP, ADD _tmp
        Becomes: LDA a, ADD b

        Similar patterns exist for SUB, AND, OR, XOR.
        """
        # Operations that can be optimized: ADD, SUB, AND, OR, XOR
        binary_ops = {'ADD', 'SUB', 'AND', 'OR', 'XOR'}

        i = 0
        optimized_count = 0
        new_elements = []

        while i < len(self.elements):
            # Look for the pattern:
            # [i]   LDA addr1
            # [i+1] LDA addr2
            # [i+2] STA _tmp
            # [i+3] POP
            # [i+4] OP _tmp  (where OP is ADD, SUB, AND, OR, XOR)

            if (i + 4 < len(self.elements) and
                isinstance(self.elements[i], Instruction) and
                isinstance(self.elements[i+1], Instruction) and
                isinstance(self.elements[i+2], Instruction) and
                isinstance(self.elements[i+3], Instruction) and
                isinstance(self.elements[i+4], Instruction)):

                instr0 = self.elements[i]
                instr1 = self.elements[i+1]
                instr2 = self.elements[i+2]
                instr3 = self.elements[i+3]
                instr4 = self.elements[i+4]

                # Check the pattern
                if (instr0.mnemonic == 'LDA' and instr0.index_reg is None and
                    instr1.mnemonic == 'LDA' and instr1.index_reg is None and
                    instr2.mnemonic == 'STA' and instr2.operand == '_tmp' and
                    instr3.mnemonic == 'POP' and
                    instr4.mnemonic in binary_ops and instr4.operand == '_tmp'):

                    # Found the pattern! Replace with optimized version
                    addr1 = instr0.operand
                    addr2 = instr1.operand
                    op = instr4.mnemonic

                    # Keep the first LDA
                    new_elements.append(instr0)

                    # Replace with direct operation on second operand
                    new_instr = Instruction(op, addr2, None, instr4.line_num)
                    new_elements.append(new_instr)

                    optimized_count += 1
                    i += 5  # Skip all 5 instructions
                    continue

            # Also handle the MUL/DIV pattern which might be different:
            # LDA a, LDA b, STA _tmp, POP, MUL/DIV
            # These use implicit _tmp so the pattern is slightly different
            if (i + 4 < len(self.elements) and
                isinstance(self.elements[i], Instruction) and
                isinstance(self.elements[i+1], Instruction) and
                isinstance(self.elements[i+2], Instruction) and
                isinstance(self.elements[i+3], Instruction) and
                isinstance(self.elements[i+4], Instruction)):

                instr0 = self.elements[i]
                instr1 = self.elements[i+1]
                instr2 = self.elements[i+2]
                instr3 = self.elements[i+3]
                instr4 = self.elements[i+4]

                # Check for MUL/DIV/MOD pattern
                # MUL/DIV use the Y register for the second operand
                if (instr0.mnemonic == 'LDA' and instr0.index_reg is None and
                    instr1.mnemonic == 'LDA' and instr1.index_reg is None and
                    instr2.mnemonic == 'TAY' and
                    instr3.mnemonic == 'POP' and
                    instr4.mnemonic in {'MUL', 'DIV', 'MOD'}):

                    # Found MUL/DIV pattern! Replace with optimized version
                    addr1 = instr0.operand
                    addr2 = instr1.operand
                    op = instr4.mnemonic

                    # Load second operand into Y
                    ldy_instr = Instruction('LDY', addr2, None, instr1.line_num)
                    new_elements.append(ldy_instr)

                    # Load first operand into AC
                    new_elements.append(instr0)

                    # Perform the operation
                    op_instr = Instruction(op, None, None, instr4.line_num)
                    new_elements.append(op_instr)

                    optimized_count += 1
                    i += 5
                    continue

            # No pattern match, keep the instruction as-is
            new_elements.append(self.elements[i])
            i += 1

        if optimized_count > 0:
            self.warnings.append(f"Peephole optimizer: {optimized_count} patterns optimized")
            self.elements = new_elements
            # Recalculate addresses after optimization
            self._recalculate_addresses()

    def _recalculate_addresses(self):
        """Recalculate addresses after peephole optimization."""
        current_address = self.origin
        for element in self.elements:
            element.address = current_address
            if isinstance(element, Instruction):
                element.size = self.get_instruction_size(element)
                current_address += element.size
            elif isinstance(element, DataDirective):
                if element.directive == '.byte':
                    element.size = len(element.values)
                elif element.directive == '.word':
                    element.size = len(element.values) * 2
                current_address += element.size

    def assemble(self, source: str) -> bool:
        """Assemble source code. Returns True on success."""
        self.pass1(source)

        if self.errors:
            return False

        # Apply peephole optimization
        self.peephole_optimize()

        self.pass2()

        return len(self.errors) == 0

    def get_binary(self, start: int = 0, end: Optional[int] = None) -> bytes:
        """Get binary output."""
        if end is None:
            end = self.max_address

        result = bytearray(end - start)
        for addr in range(start, end):
            if addr in self.memory:
                result[addr - start] = self.memory[addr]

        return bytes(result)

    def get_hex(self, start: int = 0, end: Optional[int] = None) -> str:
        """Get hex output (one byte per line with address)."""
        if end is None:
            end = self.max_address

        lines = []
        for addr in range(start, end):
            val = self.memory.get(addr, 0)
            lines.append(f"{addr:04X}: {val:02X}")

        return '\n'.join(lines)

    def get_mem(self, start: int = 0, end: Optional[int] = None) -> str:
        """Get mem output (suitable for $readmemh)."""
        if end is None:
            end = self.max_address

        lines = []
        for addr in range(start, end):
            val = self.memory.get(addr, 0)
            lines.append(f"@{addr:04X} {val:02X}")

        return '\n'.join(lines)

    def get_cocotb_list(self, start: int = 0, end: Optional[int] = None) -> List[int]:
        """Get list suitable for cocotb load_program."""
        if end is None:
            end = self.max_address

        result = []
        for addr in range(start, end):
            result.append(self.memory.get(addr, 0))

        return result

    def get_python_array(self, start: int = 0, end: Optional[int] = None) -> str:
        """Get Python list literal for cocotb."""
        data = self.get_cocotb_list(start, end)
        lines = ['[']
        for i in range(0, len(data), 16):
            chunk = data[i:i+16]
            hex_vals = ', '.join(f'0x{b:02X}' for b in chunk)
            lines.append(f'    {hex_vals},')
        lines.append(']')
        return '\n'.join(lines)


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Neander-X 16-bit Assembler',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Output formats:
  bin     Binary file (default)
  hex     Hex dump with addresses
  mem     Verilog $readmemh format
  python  Python list for cocotb

Examples:
  %(prog)s input.s -o output.bin
  %(prog)s input.s -o output.mem --format mem
  %(prog)s input.s --format python
'''
    )

    parser.add_argument('input', help='Input assembly file')
    parser.add_argument('-o', '--output', help='Output file (stdout if not specified)')
    parser.add_argument('-f', '--format', choices=['bin', 'hex', 'mem', 'python'],
                        default='bin', help='Output format (default: bin)')
    parser.add_argument('--start', type=lambda x: int(x, 0), default=0,
                        help='Start address for output (default: 0)')
    parser.add_argument('--end', type=lambda x: int(x, 0), default=None,
                        help='End address for output (default: max address)')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Verbose output')
    parser.add_argument('--symbols', action='store_true',
                        help='Print symbol table')

    args = parser.parse_args()

    # Read input
    try:
        with open(args.input, 'r') as f:
            source = f.read()
    except FileNotFoundError:
        print(f"Error: File not found: {args.input}", file=sys.stderr)
        sys.exit(1)
    except IOError as e:
        print(f"Error reading {args.input}: {e}", file=sys.stderr)
        sys.exit(1)

    # Assemble
    asm = NeanderAssembler()
    success = asm.assemble(source)

    # Print warnings
    for warning in asm.warnings:
        print(f"Warning: {warning}", file=sys.stderr)

    # Print errors
    if not success:
        for error in asm.errors:
            print(f"Error: {error}", file=sys.stderr)
        sys.exit(1)

    # Print symbol table if requested
    if args.symbols:
        print("\nSymbol Table:", file=sys.stderr)
        print("-" * 40, file=sys.stderr)
        for name, addr in sorted(asm.symbols.items(), key=lambda x: x[1]):
            print(f"  {name:20s} = 0x{addr:04X} ({addr})", file=sys.stderr)
        print("-" * 40, file=sys.stderr)

    # Print summary if verbose
    if args.verbose:
        print(f"\nAssembled {len(asm.elements)} elements", file=sys.stderr)
        print(f"Memory used: {len(asm.memory)} bytes", file=sys.stderr)
        print(f"Max address: 0x{asm.max_address:04X}", file=sys.stderr)

    # Generate output
    start = args.start
    end = args.end

    if args.format == 'bin':
        output = asm.get_binary(start, end)
    elif args.format == 'hex':
        output = asm.get_hex(start, end)
    elif args.format == 'mem':
        output = asm.get_mem(start, end)
    elif args.format == 'python':
        output = asm.get_python_array(start, end)

    # Write output
    if args.output:
        if args.format == 'bin':
            with open(args.output, 'wb') as f:
                f.write(output)
        else:
            with open(args.output, 'w') as f:
                f.write(output)
                f.write('\n')
    else:
        if args.format == 'bin':
            sys.stdout.buffer.write(output)
        else:
            print(output)

    if args.verbose:
        print(f"\nOutput written to: {args.output or 'stdout'}", file=sys.stderr)


if __name__ == '__main__':
    main()
