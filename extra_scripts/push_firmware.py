import hashlib
import os
import shutil

from SCons.Script import Import  # ty:ignore[unresolved-import]

Import("env")

def generate_ota_artifacts(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    firmware_bin = os.path.join(build_dir, "firmware.bin")
    
    # Destination in monorepo project subfolder
    bin_dir = os.path.join(env.subst("$PROJECT_DIR"), "bin")
    os.makedirs(bin_dir, exist_ok=True)
    
    dest_bin = os.path.join(bin_dir, "firmware.bin")
    dest_md5 = os.path.join(bin_dir, "firmware.md5")

    if os.path.exists(firmware_bin):
        # 1. Copy .bin
        shutil.copyfile(firmware_bin, dest_bin)
        
        # 2. Calculate MD5 checksum
        hasher = hashlib.md5()
        with open(dest_bin, "rb") as f:
            hasher.update(f.read())
        md5_hash = hasher.hexdigest()

        # 3. Write .md5 file
        with open(dest_md5, "w") as f:
            f.write(md5_hash)

        print("\n[OTA Build Post-Process] Generated:")
        print(f"  Binary: {dest_bin}")
        print(f"  MD5:    {dest_md5} ({md5_hash})\n")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", generate_ota_artifacts)  # noqa: F821  # ty:ignore[unresolved-reference]
