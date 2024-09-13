#!/usr/bin/env python3

import sys
import os

def create_file(mount_point):
    file_path = os.path.join(mount_point, "hello_from_raspberry_pi.txt")
    try:
        with open(file_path, 'w') as f:
            f.write("Hello from Raspberry Pi!")
        print(f"File created successfully at {file_path}")
    except Exception as e:
        print(f"Error creating file: {str(e)}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 script.py <mount_point>")
        sys.exit(1)
    
    mount_point = sys.argv[1]
    create_file(mount_point)