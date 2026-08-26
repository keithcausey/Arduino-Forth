#!/usr/bin/env python3
"""
Validate the generated ArdufigoROM.h file for syntax and completeness.
"""

import sys
import re

def validate_header(filename):
    """Validate the C header file"""
    errors = []
    warnings = []
    
    try:
        with open(filename, 'r') as f:
            content = f.read()
        
        # Check for proper header guards or includes
        if 'const' not in content:
            errors.append("Missing 'const' declaration")
        
        if 'PROGMEM' not in content:
            errors.append("Missing 'PROGMEM' declaration")
        
        if 'byte' not in content:
            errors.append("Missing 'byte' type")
        
        # Check for array declaration
        array_match = re.search(r'const\s+PROGMEM\s+byte\s+(\w+)\s*\[\s*0x([0-9A-Fa-f]+)\s*\]', content)
        if not array_match:
            errors.append("Could not find proper array declaration")
        else:
            array_name = array_match.group(1)
            array_size = int(array_match.group(2), 16)
            print(f"Found array: {array_name}[0x{array_size:04X}] ({array_size} bytes)")
        
        # Check for array initialization
        if '= {' not in content:
            errors.append("Missing array initialization")
        
        # Check for proper closing
        if '};' not in content:
            errors.append("Missing array closing '};'")
        
        # Count hex values
        hex_values = re.findall(r'0x[0-9A-Fa-f]{2}', content)
        print(f"Found {len(hex_values)} hex byte values")
        
        if len(hex_values) < 100:
            warnings.append("Very few hex values found - file might be incomplete")
        
        # Check for ASCII comments
        ascii_comments = re.findall(r'//.*[0-9A-Fa-f]{4}$', content, re.MULTILINE)
        print(f"Found {len(ascii_comments)} lines with ASCII representation comments")
        
        # Report results
        if errors:
            print("\nERRORS:")
            for error in errors:
                print(f"  ✗ {error}")
            return False
        
        if warnings:
            print("\nWARNINGS:")
            for warning in warnings:
                print(f"  ⚠ {warning}")
        
        print("\n✓ Header file validation passed!")
        return True
        
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
        return False
    except Exception as e:
        print(f"Error validating file: {e}")
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: validate_header.py <header_file>")
        sys.exit(1)
    
    filename = sys.argv[1]
    print(f"Validating {filename}...")
    print("=" * 60)
    
    if validate_header(filename):
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
