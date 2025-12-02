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

Import("env")

# Path to the .env file (relative to project root)
env_file = os.path.join(env.subst("$PROJECT_DIR"), ".env")


def parse_env_file(filepath):
    """Parse a .env file and return a dictionary of key-value pairs."""
    config = {}
    if not os.path.exists(filepath):
        return config

    with open(filepath, "r") as f:
        for line in f:
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
                    config[key] = value
    return config


def get_build_flag(key, value):
    """
    Convert a key-value pair to a PlatformIO build flag.

    - String values are wrapped in escaped quotes
    - Boolean values (true/false) are converted to lowercase
    - Integer values are passed as-is
    """
    # The config.cpp expects *_VALUE macros
    macro_name = f"{key}_VALUE"

    # Determine the type of value and format accordingly
    if value.lower() in ("true", "false"):
        # Boolean value
        return f"-D{macro_name}={value.lower()}"
    elif value.isdigit() or (value.startswith("-") and value[1:].isdigit()):
        # Integer value
        return f"-D{macro_name}={value}"
    else:
        # String value - wrap in escaped quotes
        # Escape any backslashes and quotes in the value
        escaped = value.replace("\\", "\\\\").replace('"', '\\"')
        return f'-D{macro_name}=\\"{escaped}\\"'


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
