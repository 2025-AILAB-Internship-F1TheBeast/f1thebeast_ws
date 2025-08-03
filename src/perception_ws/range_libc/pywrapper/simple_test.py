#!/usr/bin/env python3
import numpy as np

try:
    import range_libc
    print("Successfully imported range_libc!")
    
    # Create a simple test map
    test_map = np.ones((100, 100), dtype=np.float32) * 255  # white map
    test_map[40:60, 40:60] = 0  # black square in middle
    
    # Create OMap
    omap = range_libc.PyOMap(test_map)
    print("Successfully created OMap")
    
    print("range_libc compilation and basic functionality test passed!")
    
except ImportError as e:
    print(f"Import error: {e}")
except Exception as e:
    print(f"Error: {e}")