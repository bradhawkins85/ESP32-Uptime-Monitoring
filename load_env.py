"""
PlatformIO extra script to load environment variables from a .env file.

This script reads configuration values from a .env file and converts them
to PlatformIO build flags, allowing users to store sensitive configuration
outside of source files that might be committed to git.

Usage:
    1. Copy .env.example to .env
    2. Edit .env with your configuration values
    3. Build normally with PlatformIO - values are automatically loaded
"""

import os
import re

Import("env")

# Path to the .env file (relative to project root)
env_file = os.path.join(env.subst("$PROJECT_DIR"), ".env")

# Valid environment variable key pattern (alphanumeric and underscores)
KEY_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def parse_env_file(filepath):
    """Parse a .env file and return a dictionary of key-value pairs."""
    config = {}
    if not os.path.exists(filepath):
        return config

    with open(filepath, "r") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            # Skip empty lines and comments
            if not line or line.startswith("#"):
                continue
            # Parse key=value pairs
            if "=" in line:
                key, _, value = line.partition("=")
                key = key.strip()
                value = value.strip()
                if key:
                    if not KEY_PATTERN.match(key):
                        print(f"  Warning: Skipping invalid key '{key}' on line {line_num}")
                        continue
                    config[key] = value
    return config


def get_build_flag(key, value):
    """
    Convert a key-value pair to a PlatformIO build flag.

    - String values are wrapped in escaped quotes
    - Boolean values (true/false) are converted to lowercase
    - Integer values are passed as-is
    - Floating point values are passed as-is
    """
    # The config.cpp expects *_VALUE macros
    macro_name = f"{key}_VALUE"

    # Determine the type of value and format accordingly
    if value.lower() in ("true", "false"):
        # Boolean value
        return f"-D{macro_name}={value.lower()}"
    elif value.isdigit() or (len(value) > 1 and value.startswith("-") and value[1:].isdigit()):
        # Integer value (positive or negative)
        return f"-D{macro_name}={value}"
    elif is_float(value):
        # Floating point value
        return f"-D{macro_name}={value}"
    else:
        # String value - wrap in escaped quotes
        # Escape backslashes, double quotes, single quotes, and spaces for the shell
        # Double quotes need triple escaping to survive shell -> compiler -> C string
        escaped = value.replace("\\", "\\\\").replace('"', '\\\\\\"').replace("'", "\\'").replace(" ", "\\ ")
        return f'-D{macro_name}=\\"{escaped}\\"'


def is_float(value):
    """Check if a string represents a floating point number."""
    try:
        float(value)
        # Make sure it's not just an integer (those are handled separately)
        return "." in value or "e" in value.lower()
    except ValueError:
        return False


# Load configuration from .env file
config = parse_env_file(env_file)

if config:
    print(f"Loading configuration from {env_file}")

    # Build the list of flags
    new_flags = []
    for key, value in config.items():
        flag = get_build_flag(key, value)
        new_flags.append(flag)
        # Only show non-sensitive keys in output
        if any(
            sensitive in key.upper()
            for sensitive in ["PASSWORD", "TOKEN", "SECRET", "WEBHOOK"]
        ):
            print(f"  {key}: ****")
        else:
            print(f"  {key}: {value}")

    # Append the new flags to existing build flags
    env.Append(BUILD_FLAGS=new_flags)
else:
    print(
        f"No .env file found at {env_file}. Using default values from config.cpp or build_flags in platformio.ini."
    )
