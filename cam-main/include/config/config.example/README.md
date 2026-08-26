# Configuration Templates

This directory contains sanitized templates and default schemas for this module.

> **Security Notice:**
> - Files in this folder are tracked in version control. **Never commit real credentials, passwords, or private keys here.**
> - Active runtime configuration files live in the parent directory (`../`) and are ignored by Git (`.gitignore`).

---

## Quick Setup (Bootstrap)

To initialize active configs from these templates without overwriting existing files, run the command for your OS **from inside this directory**:

### Linux / macOS (Bash)
```bash
for f in *.example *.*.example; do
  [ -f "$f" ] && cp -n "$f" "../${f%.example}" && echo "Created ../${f%.example}"
done
```

### Windows (PowerShell)
```powershell
Get-ChildItem -File -Filter "*.example" | ForEach-Object {
    $dest = Join-Path ".." ($_.Name -replace '\.example$', '')
    if (-not (Test-Path $dest)) {
        Copy-Item $_.FullName -Destination $dest
        Write-Host "Created: $dest" -ForegroundColor Green
    } else {
        Write-Host "Skipped (Exists): $dest" -ForegroundColor Yellow
    }
}
```

---

## Manual Setup

1. Copy the target `<filename>.example` file to the parent folder (`../`).
2. Remove the `.example` suffix so the filename matches the active configuration name:
   - `config.h.example` -> `../config.h`
   - `settings.env.example` -> `../settings.env`
   - `network.conf.example` -> `../network.conf`
3. Open the active file in `../` and replace all placeholder values (`YOUR_API_KEY`, `WIFI_SSID`, `0.0.0.0`, etc.) with your local credentials.

---

## Adding a New Template

When adding new config parameters or files:
1. Update your local config in `../`.
2. Duplicate the file into this folder and append `.example`.
3. Strip all sensitive data and replace with generic placeholders before committing.