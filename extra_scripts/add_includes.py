Import("env")
import os

# Recursively append 'include' and all child folders to PlatformIO's include path
for root, dirs, files in os.walk("include"):
    env.Append(CPPPATH=[os.path.abspath(root)])