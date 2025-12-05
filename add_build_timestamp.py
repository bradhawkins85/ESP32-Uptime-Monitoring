Import("env")
from datetime import datetime

# Get current date and time
build_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

# Add as a build flag
env.Append(CPPDEFINES=[
    ("BUILD_TIMESTAMP", f'\\"{build_time}\\"')
])

print(f"Build timestamp: {build_time}")
