#!/usr/bin/env python3
"""
TASM-compatible assembler for Arduino-Forth project.
This script replaces the proprietary TASM assembler with an open-source alternative.

Supports TASM directives used in Ardufigo.ASM:
- .dw (define word - 16-bit little-endian)
- .db (define byte)
- .TEXT (define text/string)
- .ORG (set origin address)
- .equ (define constant)
- .MSFIRST (little-endian mode)
- .BYTE (define single byte)
- .END (end of assembly)

Author: Open Source Replacement
License: Public Domain (matching FIG-FORTH release)
"""

import sys
import re
import struct
from typing import Dict, List, Tuple, Optional

class TASMAssembler:
    def __init__(self):
        self.symbols: Dict[str, int] = {}
        self.output: bytearray = bytearray(0x4000)  # 16KB
        self.pc: int = 0  # Program counter
        self.little_endian: bool = True
        self.labels: Dict[str, int] = {}
        
    def parse_number(self, s: str) -> int:
        """Parse number in various formats (hex, decimal, binary)"""
        s = s.strip()
        if s.endswith('h'):
            # Hex number ending with 'h' (like 0ah, 3Ch)
            return int(s[:-1], 16)
        elif s.startswith('0x') or s.startswith('0X'):
            return int(s, 16)
        elif s.startswith('0b') or s.startswith('0B'):
            return int(s, 2)
        elif s.startswith('"') and s.endswith('"'):
            # String literal - return first character code
            return ord(s[1]) if len(s) > 2 else 0
        else:
            # Try to parse as decimal, might be negative
            try:
                return int(s, 10)
            except ValueError:
                # Might be a symbol - return 0 as placeholder
                return 0
    
    def evaluate_expression(self, expr: str) -> int:
        """Evaluate simple arithmetic expressions"""
        expr = expr.strip()
        
        # Handle $ (current address) - needs to be done before symbol replacement
        if expr == '$+2':
            return self.pc + 2
        expr = expr.replace('$', str(self.pc))
        
        # Replace symbols with their values
        for symbol, value in sorted(self.symbols.items(), key=lambda x: -len(x[0])):
            # Use word boundaries to avoid partial matches
            expr = re.sub(r'\b' + re.escape(symbol) + r'\b', str(value), expr)
        
        # Try to evaluate arithmetic expressions
        try:
            # Handle hex numbers with 'h' suffix like '0ah', '3Ch'
            expr = re.sub(r'\b([0-9a-fA-F]+)h\b', lambda m: str(int(m.group(1), 16)), expr)
            
            # Simple evaluation for basic arithmetic
            result = eval(expr, {"__builtins__": {}}, {})
            return int(result)
        except:
            # If evaluation fails, try to parse as number
            try:
                return self.parse_number(expr)
            except:
                # If it's still a symbol (forward reference), return placeholder
                return 0
    
    def write_byte(self, value: int, address: Optional[int] = None):
        """Write a byte to output"""
        if address is None:
            address = self.pc
        self.output[address] = value & 0xFF
        if address == self.pc:
            self.pc += 1
    
    def write_word(self, value: int, address: Optional[int] = None):
        """Write a 16-bit word to output (little-endian)"""
        if address is None:
            address = self.pc
        if self.little_endian:
            self.output[address] = value & 0xFF
            self.output[address + 1] = (value >> 8) & 0xFF
        else:
            self.output[address] = (value >> 8) & 0xFF
            self.output[address + 1] = value & 0xFF
        if address == self.pc:
            self.pc += 2
    
    def write_string(self, text: str):
        """Write a string to output"""
        for char in text:
            self.write_byte(ord(char))
    
    def process_line(self, line: str, line_num: int):
        """Process a single line of assembly"""
        # Remove comments (starting with ;)
        if ';' in line:
            line = line[:line.index(';')]
        
        line = line.strip()
        if not line:
            return
        
        # Check for label (ends with :)
        if ':' in line and not line.startswith('.'):
            parts = line.split(':', 1)
            label = parts[0].strip()
            self.labels[label] = self.pc
            self.symbols[label] = self.pc
            if len(parts) > 1:
                line = parts[1].strip()
            else:
                return
        
        # Process directives
        if line.startswith('.MSFIRST') or line.upper().startswith('.MSFIRST'):
            self.little_endian = True
            return
        
        if line.upper().startswith('.ORG'):
            match = re.match(r'\.ORG\s+(\S+)', line, re.IGNORECASE)
            if match:
                self.pc = self.evaluate_expression(match.group(1))
            return
        
        if line.startswith('.equ') or line.upper().startswith('EQU') or ' equ ' in line.lower() or ' .equ ' in line.lower():
            # Format: symbol .equ value or symbol equ value
            match = re.match(r'(\w+)\s+(?:\.equ|equ)\s+(.+)', line, re.IGNORECASE)
            if match:
                symbol = match.group(1)
                value = self.evaluate_expression(match.group(2))
                self.symbols[symbol] = value
            return
        
        if line.startswith('#define'):
            # #define symbol value - just skip, handled by preprocessing
            return
        
        if line.upper().startswith('.PAGE'):
            # Page directive - ignore for now
            return
        
        if line.upper().startswith('.END'):
            # End of assembly
            return
        
        # Data directives
        if line.upper().startswith('DW') or line.upper().startswith('.DW'):
            # Define word(s)
            match = re.match(r'(?:DW|\.dw)\s+(.+)', line, re.IGNORECASE)
            if match:
                values = match.group(1).split(',')
                for val in values:
                    val = val.strip()
                    if val:
                        self.write_word(self.evaluate_expression(val))
            return
        
        if line.upper().startswith('DEFB') or line.upper().startswith('.DB'):
            # Define byte(s)
            match = re.match(r'(?:defb|\.db)\s+(.+)', line, re.IGNORECASE)
            if match:
                values = match.group(1).split(',')
                for val in values:
                    val = val.strip()
                    if val:
                        # Handle string literals like "EXECUTE"
                        if val.startswith('"') and val.endswith('"'):
                            self.write_string(val[1:-1])
                        else:
                            self.write_byte(self.evaluate_expression(val))
            return
        
        if line.upper().startswith('DEFC') or line.upper().startswith('.TEXT'):
            # Define text/string
            match = re.match(r'(?:defc|\.TEXT)\s+"([^"]*)"', line, re.IGNORECASE)
            if match:
                text = match.group(1)
                self.write_string(text)
            return
        
        if line.upper().startswith('.BYTE'):
            # Define byte(s)
            match = re.match(r'\.BYTE\s+(.+)', line, re.IGNORECASE)
            if match:
                values = match.group(1).split(',')
                for val in values:
                    val = val.strip()
                    if val.startswith('"') and val.endswith('"'):
                        # String literal
                        self.write_string(val[1:-1])
                    elif val:
                        self.write_byte(self.evaluate_expression(val))
            return
    
    def preprocess(self, lines: List[str]) -> List[str]:
        """Preprocess the assembly file to handle #define macros"""
        defines = {}
        processed = []
        
        for line in lines:
            # Handle #define
            match = re.match(r'#define\s+(\w+)\s+(.+)', line)
            if match:
                defines[match.group(1)] = match.group(2).strip()
                processed.append(line)  # Keep for reference
                continue
            
            # Replace defined symbols
            for symbol, replacement in defines.items():
                line = re.sub(r'\b' + re.escape(symbol) + r'\b', replacement, line)
            
            processed.append(line)
        
        return processed
    
    def assemble(self, input_file: str, output_file: str):
        """Assemble the input file and write output"""
        # Read input file
        with open(input_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
        
        # Preprocess
        lines = self.preprocess(lines)
        
        # Pass 1: Collect all labels and .equ symbols
        print("Pass 1: Collecting symbols...")
        saved_pc = self.pc
        for i, line in enumerate(lines, 1):
            try:
                # Only collect labels and symbols, don't write data
                orig_line = line
                if ';' in line:
                    line = line[:line.index(';')]
                line = line.strip()
                
                # Handle labels
                if ':' in line and not line.startswith('.'):
                    parts = line.split(':', 1)
                    label = parts[0].strip()
                    self.labels[label] = self.pc
                    self.symbols[label] = self.pc
                    if len(parts) > 1:
                        line = parts[1].strip()
                
                # Handle .equ
                if line.startswith('.equ') or line.upper().startswith('EQU') or ' equ ' in line.lower():
                    match = re.match(r'(\w+)\s+(?:\.equ|equ)\s+(.+)', line, re.IGNORECASE)
                    if match:
                        symbol = match.group(1)
                        # Try to evaluate, but if it fails (forward reference), skip for now
                        try:
                            value = self.evaluate_expression(match.group(2))
                            self.symbols[symbol] = value
                        except:
                            pass
                
                # Track PC for data directives
                if line.upper().startswith('.ORG'):
                    match = re.match(r'\.ORG\s+(\S+)', line, re.IGNORECASE)
                    if match:
                        try:
                            self.pc = self.evaluate_expression(match.group(1))
                        except:
                            pass
                elif line.upper().startswith('DW') or line.upper().startswith('.DW'):
                    match = re.match(r'(?:DW|\.dw)\s+(.+)', line, re.IGNORECASE)
                    if match:
                        count = len([v.strip() for v in match.group(1).split(',') if v.strip()])
                        self.pc += count * 2
                elif line.upper().startswith('DEFB') or line.upper().startswith('.DB'):
                    match = re.match(r'(?:defb|\.db)\s+(.+)', line, re.IGNORECASE)
                    if match:
                        values = [v.strip() for v in match.group(1).split(',') if v.strip()]
                        for val in values:
                            if val.startswith('"') and val.endswith('"'):
                                self.pc += len(val) - 2
                            else:
                                self.pc += 1
                elif line.upper().startswith('DEFC') or line.upper().startswith('.TEXT'):
                    match = re.match(r'(?:defc|\.TEXT)\s+"([^"]*)"', line, re.IGNORECASE)
                    if match:
                        self.pc += len(match.group(1))
                elif line.upper().startswith('.BYTE'):
                    match = re.match(r'\.BYTE\s+(.+)', line, re.IGNORECASE)
                    if match:
                        values = [v.strip() for v in match.group(1).split(',') if v.strip()]
                        for val in values:
                            if val.startswith('"') and val.endswith('"'):
                                self.pc += len(val) - 2
                            else:
                                self.pc += 1
            except Exception as e:
                # Silently continue in pass 1
                pass
        
        print(f"  Found {len(self.symbols)} symbols and {len(self.labels)} labels")
        
        # Pass 2: Generate code
        print("Pass 2: Generating code...")
        self.pc = 0
        self.output = bytearray(0x4000)
        for i, line in enumerate(lines, 1):
            try:
                self.process_line(line, i)
            except Exception as e:
                print(f"Warning on line {i}: {e}", file=sys.stderr)
        
        # Write output as Intel HEX format
        self.write_intel_hex(output_file)
    
    def write_intel_hex(self, output_file: str):
        """Write output in Intel HEX format"""
        with open(output_file, 'w') as f:
            # Write data in 16-byte records
            addr = 0
            while addr < len(self.output):
                # Find end of non-FF data
                if addr >= 0x3FFF:
                    break
                
                # Get record length (up to 16 bytes)
                length = min(16, 0x4000 - addr)
                
                # Check if this record contains any non-FF data
                record_data = self.output[addr:addr+length]
                if all(b == 0xFF for b in record_data):
                    addr += length
                    continue
                
                # Build record: length, address, type (00 = data)
                record = f":{length:02X}{addr:04X}00"
                checksum = length + (addr >> 8) + (addr & 0xFF)
                
                # Add data bytes
                for byte in record_data:
                    record += f"{byte:02X}"
                    checksum += byte
                
                # Add checksum (two's complement)
                checksum = (~checksum + 1) & 0xFF
                record += f"{checksum:02X}\n"
                
                f.write(record)
                addr += length
            
            # Write EOF record
            f.write(":00000001FF\n")

def main():
    if len(sys.argv) != 3:
        print("Usage: tasm_compat.py <input.asm> <output.hex>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    assembler = TASMAssembler()
    assembler.assemble(input_file, output_file)
    print(f"Assembly complete: {output_file}")

if __name__ == "__main__":
    main()
