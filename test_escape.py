# Test different escaping methods
import os
import sys

value = 'Test Network'

# Method 1: escape backslash first, then quotes, then spaces
escaped = value.replace("\\", "\\\\").replace('"', '\\"').replace(' ', '\\ ')
flag1 = f'-DWIFI_SSID_VALUE="{escaped}"'
print('Method 1:', repr(flag1))

# Method 2: wrap entire thing in single quotes (PlatformIO specific)
escaped2 = value.replace("\\", "\\\\").replace('"', '\\"')
flag2 = f"'-DWIFI_SSID_VALUE=\"{escaped2}\"'"
print('Method 2:', repr(flag2))

