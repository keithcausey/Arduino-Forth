#!/usr/bin/env python3
"""
Basic C/C++ syntax checker for Arduino code.
This performs basic validation without requiring a full compiler.
"""

import sys
import re

def check_arduino_syntax(ino_file, header_file):
    """Basic syntax checking for Arduino .ino file"""
    errors = []
    warnings = []
    
    try:
        # Read the .ino file
        with open(ino_file, 'r') as f:
            ino_content = f.read()
        
        # Read the header file
        with open(header_file, 'r') as f:
            header_content = f.read()
        
        print(f"Checking {ino_file}...")
        print("=" * 60)
        
        # Check if header is included
        if 'ArdufigoROM.h' not in ino_content:
            errors.append("ArdufigoROM.h not included in .ino file")
        else:
            print("✓ ArdufigoROM.h is included")
        
        # Check for required includes
        required_includes = ['pgmspace.h', 'EEPROM.h', 'math.h', 'SD.h']
        for inc in required_includes:
            if inc in ino_content:
                print(f"✓ {inc} is included")
            else:
                warnings.append(f"{inc} not found (might be commented out)")
        
        # Check if body array is declared in header
        if 'const PROGMEM byte body' in header_content:
            print("✓ body array declared in header")
        else:
            errors.append("body array not found in header")
        
        # Check if body array is referenced in .ino
        if 'body[' in ino_content or 'body +' in ino_content:
            print("✓ body array is referenced in code")
        else:
            warnings.append("body array not obviously referenced (might be indirect)")
        
        # Check for balanced braces
        open_braces = ino_content.count('{')
        close_braces = ino_content.count('}')
        if open_braces == close_braces:
            print(f"✓ Balanced braces ({open_braces} pairs)")
        else:
            errors.append(f"Unbalanced braces: {open_braces} {{ vs {close_braces} }}")
        
        # Check for balanced parentheses (rough check)
        open_parens = ino_content.count('(')
        close_parens = ino_content.count(')')
        if open_parens == close_parens:
            print(f"✓ Balanced parentheses ({open_parens} pairs)")
        else:
            warnings.append(f"Possibly unbalanced parentheses: {open_parens} ( vs {close_parens} )")
        
        # Check for setup() and loop() functions
        if 'void setup(' in ino_content:
            print("✓ setup() function found")
        else:
            errors.append("setup() function not found")
        
        if 'void loop(' in ino_content:
            print("✓ loop() function found")
        else:
            errors.append("loop() function not found")
        
        # Report results
        print("\n" + "=" * 60)
        if errors:
            print("\nERRORS:")
            for error in errors:
                print(f"  ✗ {error}")
        
        if warnings:
            print("\nWARNINGS:")
            for warning in warnings:
                print(f"  ⚠ {warning}")
        
        if not errors:
            print("\n✓ Basic syntax validation passed!")
            print("\nNote: This is a basic check. Full compilation with Arduino IDE")
            print("or arduino-cli is recommended for complete validation.")
            return True
        else:
            print("\n✗ Validation failed with errors")
            return False
            
    except FileNotFoundError as e:
        print(f"Error: File not found - {e}")
        return False
    except Exception as e:
        print(f"Error during validation: {e}")
        return False

def main():
    if len(sys.argv) != 3:
        print("Usage: check_arduino.py <sketch.ino> <header.h>")
        print("Example: check_arduino.py Ardufigo/Ardufigo.ino Ardufigo/ArdufigoROM.h")
        sys.exit(1)
    
    ino_file = sys.argv[1]
    header_file = sys.argv[2]
    
    if check_arduino_syntax(ino_file, header_file):
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
