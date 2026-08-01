import hashlib
import os
import shutil
import socket
import subprocess
import tempfile

Import("env")  # ty:ignore[unresolved-reference]  # noqa: F821


def _as_bool(raw, default=False):
    if raw is None:
        return default
    return str(raw).strip().lower() in {"1", "true", "yes", "on"}


def _get_project_option(name: str, default=""):
    try:
        return env.GetProjectOption(name, default)  # noqa: F821  # ty:ignore[unresolved-reference]
    except Exception:
        return default


def _read_setting(project_option: str, env_name: str, default=""):
    raw_env = os.getenv(env_name)
    if raw_env is not None:
        return raw_env
    return _get_project_option(project_option, default)


def _read_bool_setting(project_option: str, env_name: str, default=False):
    raw_env = os.getenv(env_name)
    if raw_env is not None:
        return _as_bool(raw_env, default)
    return _as_bool(_get_project_option(project_option, default), default)


def _read_int_setting(project_option: str, env_name: str, default=0) -> int:
    raw = _read_setting(project_option, env_name, default)
    try:
        return int(str(raw).strip())
    except (TypeError, ValueError):
        return int(default)


def _artifact_path() -> str:
    raw = str(
        _read_setting("custom_firmware_artifact", "PUSH_FIRMWARE_ARTIFACT", "firmware")
    ).strip()

    if os.path.isabs(raw):
        raise ValueError(f"Firmware artifact path must be relative: {raw!r}")

    artifact = (raw or "firmware").replace("\\", "/").strip("/")

    for suffix in (".bin", ".md5"):
        if artifact.endswith(suffix):
            artifact = artifact[: -len(suffix)]

    parts = [part for part in artifact.split("/") if part]
    if not parts or any(part in {".", ".."} for part in parts):
        raise ValueError(f"Invalid firmware artifact path: {raw!r}")

    return "/".join(parts)


def _artifact_fs_path(base_dir: str, artifact_file: str) -> str:
    return os.path.join(base_dir, *artifact_file.split("/"))


def _ensure_parent_dir(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def _ota_target_from_artifact(artifact: str) -> str:
    name = os.path.basename(artifact)
    if name == "firmware-esp32s3":
        return "esp32s3"
    if name in {"firmware", "firmware-esp32cam"}:
        return "esp32cam"
    return name


def _mqtt_remaining_length(length: int) -> bytes:
    encoded = bytearray()
    while True:
        byte = length % 128
        length //= 128
        if length:
            byte |= 0x80
        encoded.append(byte)
        if not length:
            return bytes(encoded)


def _mqtt_utf8(value: str) -> bytes:
    data = value.encode("utf-8")
    return len(data).to_bytes(2, "big") + data


def _publish_mqtt_stdlib(
    host: str,
    port: int,
    topic: str,
    payload: str,
    retain: bool,
) -> None:
    client_id = f"pio-ota-{os.getpid()}"
    connect_payload = _mqtt_utf8(client_id)
    connect_variable = (
        _mqtt_utf8("MQTT")
        + bytes([4, 0x02])
        + (30).to_bytes(2, "big")
    )
    connect_packet = (
        bytes([0x10])
        + _mqtt_remaining_length(len(connect_variable) + len(connect_payload))
        + connect_variable
        + connect_payload
    )

    publish_payload = payload.encode("utf-8")
    publish_variable = _mqtt_utf8(topic)
    publish_header = 0x30 | (0x01 if retain else 0)
    publish_packet = (
        bytes([publish_header])
        + _mqtt_remaining_length(len(publish_variable) + len(publish_payload))
        + publish_variable
        + publish_payload
    )

    with socket.create_connection((host, port), timeout=10) as sock:
        sock.sendall(connect_packet)
        connack = sock.recv(4)
        if connack != b"\x20\x02\x00\x00":
            raise RuntimeError(f"MQTT CONNACK rejected: {connack!r}")
        sock.sendall(publish_packet)
        sock.sendall(b"\xe0\x00")


def _file_md5(path: str) -> str:
    digest = hashlib.md5()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git(project_dir: str, *args: str):
    return subprocess.run(
        ["git", *args],
        cwd=project_dir,
        check=True,
        text=True,
        capture_output=True,
    )


def _git_out(project_dir: str, *args: str) -> str:
    return _git(project_dir, *args).stdout.strip()


def _current_branch(project_dir: str) -> str:
    try:
        return _git_out(project_dir, "branch", "--show-current")
    except subprocess.CalledProcessError:
        return ""


def _target_branch(project_dir: str, configured_branch: str) -> str:
    if configured_branch:
        return configured_branch
    return _current_branch(project_dir) or "main"


def _fetch_remote_branch(project_dir: str, remote: str, branch: str) -> None:
    _git(project_dir, "fetch", remote, f"{branch}:refs/remotes/{remote}/{branch}")


def _remote_file_text(project_dir: str, remote: str, branch: str, path: str) -> str:
    try:
        return _git_out(project_dir, "show", f"{remote}/{branch}:{path}")
    except subprocess.CalledProcessError:
        return ""


def _remove_worktree(project_dir: str, worktree_dir: str, temp_root: str) -> None:
    try:
        _git(project_dir, "worktree", "remove", "--force", worktree_dir)
    except Exception:
        pass
    shutil.rmtree(temp_root, ignore_errors=True)


def _fast_forward_local_if_possible(project_dir: str, remote: str, branch: str) -> None:
    current = _current_branch(project_dir)
    if current != branch:
        print(
            f"[OTA] Remote {remote}/{branch} updated. Local branch '{current}' "
            "was not moved because it is not the target branch."
        )
        return

    try:
        _git(project_dir, "merge", "--ff-only", f"{remote}/{branch}")
        print(f"[OTA] Local {branch} fast-forwarded to {remote}/{branch}.")
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr.strip() if exc.stderr else str(exc)
        print(
            f"[OTA] Remote {remote}/{branch} updated, but local {branch} could "
            f"not be fast-forwarded automatically: {stderr}"
        )


def _publish_mqtt_trigger(artifact: str) -> None:
    enabled = _read_bool_setting(
        "custom_ota_trigger_mqtt",
        "PUSH_FIRMWARE_TRIGGER_MQTT",
        False,
    )
    topic = str(
        _read_setting("custom_ota_trigger_topic", "PUSH_FIRMWARE_TRIGGER_TOPIC", "")
    ).strip()
    if not enabled or not topic:
        return

    host = (
        str(
            _read_setting(
                "custom_ota_trigger_host",
                "PUSH_FIRMWARE_TRIGGER_HOST",
                "broker.hivemq.com",
            )
        ).strip()
        or "broker.hivemq.com"
    )
    port = _read_int_setting("custom_ota_trigger_port", "PUSH_FIRMWARE_TRIGGER_PORT", 1883)
    payload = str(
        _read_setting("custom_ota_trigger_payload", "PUSH_FIRMWARE_TRIGGER_PAYLOAD", "1")
    )
    retain = _read_bool_setting(
        "custom_ota_trigger_retain",
        "PUSH_FIRMWARE_TRIGGER_RETAIN",
        True,
    )

    used_fallback = False
    try:
        import paho.mqtt.publish as publish
    except ImportError:
        used_fallback = True
        publish = None  # ty:ignore[invalid-assignment]

    try:
        if publish:
            publish.single(
                topic,
                payload,
                hostname=host,
                port=port,
                qos=1,
                retain=retain,
            )
        else:
            _publish_mqtt_stdlib(host, port, topic, payload, retain)
    except Exception as exc:
        print(f"[OTA] MQTT trigger failed for {topic}: {exc}")
        return

    retained = " retained" if retain else ""
    fallback = " via stdlib MQTT fallback" if used_fallback else ""
    print(
        f"[OTA] Published{retained} MQTT trigger for "
        f"{_ota_target_from_artifact(artifact)}: {topic}{fallback}"
    )


def _publish_via_temp_worktree(
    project_dir: str,
    bin_src: str,
    firmware_name: str,
    md5_name: str,
    new_md5: str,
    remote: str,
    branch: str,
    commit_message: str,
) -> None:
    temp_root = tempfile.mkdtemp(prefix="pio-ota-publish-")
    worktree_dir = os.path.join(temp_root, "repo")

    try:
        _git(project_dir, "worktree", "add", "--detach", worktree_dir, f"{remote}/{branch}")

        firmware_dst = _artifact_fs_path(worktree_dir, firmware_name)
        md5_dst = _artifact_fs_path(worktree_dir, md5_name)
        _ensure_parent_dir(firmware_dst)
        _ensure_parent_dir(md5_dst)

        shutil.copy2(bin_src, firmware_dst)
        with open(md5_dst, "w", encoding="utf-8") as handle:
            handle.write(new_md5)

        _git(worktree_dir, "add", "-f", firmware_name, md5_name)
        _git(worktree_dir, "commit", "-m", commit_message)
        commit_sha = _git_out(worktree_dir, "rev-parse", "--short", "HEAD")
        _git(worktree_dir, "push", remote, f"HEAD:{branch}")
        print(f"[OTA] Published {firmware_name} as {commit_sha} on {remote}/{branch}.")
    finally:
        _remove_worktree(project_dir, worktree_dir, temp_root)


def push_firmware(source, target, env):
    project_dir = env.subst("$PROJECT_DIR")
    bin_src = env.subst("$BUILD_DIR/${PROGNAME}.bin")
    if not os.path.exists(bin_src) and target:
        bin_src = str(target[0])
    if not os.path.exists(bin_src):
        print(f"\n[OTA] Firmware image not found yet: {bin_src}\n")
        return
    try:
        artifact = _artifact_path()
    except ValueError as exc:
        print(f"\n[OTA] {exc}\n")
        return

    firmware_name = f"{artifact}.bin"
    md5_name = f"{artifact}.md5"
    firmware_bin = _artifact_fs_path(project_dir, firmware_name)
    md5_file = _artifact_fs_path(project_dir, md5_name)

    new_md5 = _file_md5(bin_src)
    dry_run = _read_bool_setting(
        "custom_dry_run_firmware",
        "PUSH_FIRMWARE_DRY_RUN",
        False,
    )
    auto_push = _read_bool_setting(
        "custom_auto_push_firmware",
        "PUSH_FIRMWARE_ON_BUILD",
        False,
    )
    remote = (
        str(
            _read_setting("custom_auto_push_remote", "PUSH_FIRMWARE_REMOTE", "origin")
        ).strip()
        or "origin"
    )
    branch = str(
        _read_setting("custom_auto_push_branch", "PUSH_FIRMWARE_BRANCH", "")
    ).strip()
    branch = _target_branch(project_dir, branch)
    commit_prefix = (
        str(
            _read_setting(
                "custom_auto_push_commit_prefix",
                "PUSH_FIRMWARE_COMMIT_PREFIX",
                "OTA",
            )
        ).strip()
        or "OTA"
    )
    commit_message = f"{commit_prefix}: {artifact} {new_md5[:8]}"

    old_md5 = ""
    if auto_push:
        try:
            _fetch_remote_branch(project_dir, remote, branch)
            old_md5 = _remote_file_text(project_dir, remote, branch, md5_name).strip()
        except subprocess.CalledProcessError as exc:
            stderr = exc.stderr.strip() if exc.stderr else str(exc)
            print(f"[OTA] Could not read {remote}/{branch}: {stderr}\n")
            return
    elif os.path.exists(md5_file):
        with open(md5_file, "r", encoding="utf-8") as handle:
            old_md5 = handle.read().strip()

    if new_md5 == old_md5:
        print(f"\n[OTA] {firmware_name} unchanged on {remote}/{branch}; skipping publish.\n")
        return

    if dry_run:
        print(
            f"\n[OTA] Dry run: {firmware_name} would update {remote}/{branch} "
            f"from {old_md5 or '<none>'} to {new_md5}.\n"
        )
        return

    if not auto_push:
        _ensure_parent_dir(firmware_bin)
        _ensure_parent_dir(md5_file)

        shutil.copy2(bin_src, firmware_bin)
        with open(md5_file, "w", encoding="utf-8") as handle:
            handle.write(new_md5)
        print(
            f"\n[OTA] Updated {firmware_name} and {md5_name} locally. "
            "Enable custom_auto_push_firmware or set PUSH_FIRMWARE_ON_BUILD=1 "
            "to publish to GitHub.\n"
        )
        return

    print(f"\n[OTA] New {firmware_name} detected for {remote}/{branch}. MD5: {new_md5}")

    try:
        _publish_via_temp_worktree(
            project_dir,
            bin_src,
            firmware_name,
            md5_name,
            new_md5,
            remote,
            branch,
            commit_message,
        )
        _fetch_remote_branch(project_dir, remote, branch)
        _fast_forward_local_if_possible(project_dir, remote, branch)
        _publish_mqtt_trigger(artifact)
        print(f"[OTA] ESP32 will update from {remote}/{branch}.\n")
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr.strip() if exc.stderr else str(exc)
        print(f"[OTA] Git auto-push failed: {stderr}\n")


if not env.IsIntegrationDump():  # noqa: F821  # ty:ignore[unresolved-reference]
    env.AddPostAction("buildprog", push_firmware)  # noqa: F821  # ty:ignore[unresolved-reference]
