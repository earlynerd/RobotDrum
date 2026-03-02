# Codex Workspace Setup

Use these commands from the repo root on a fresh workspace:

```powershell
pwsh -File scripts/bootstrap.ps1 -Recreate -BasePython "C:\Program Files\Python312\python.exe"
pwsh -File scripts/preflight.ps1
```

If you do not have a global Python install, pass another absolute path that is readable by sandboxed users, for example:

```powershell
pwsh -File scripts/bootstrap.ps1 -Recreate -BasePython "C:\Program Files\KiCad\9.0\bin\python.exe"
```

If bootstrap fails with a temp-directory permission error, pick a different interpreter path. In this environment, Python 3.14 may fail sandbox temp checks; Python 3.12/3.11 works.

Then run tools through the wrapper:

```powershell
pwsh -File scripts/codex-run-tool.ps1 pio run -e pico32
pwsh -File scripts/codex-run-tool.ps1 pio run -t upload -e pico32
```

## Notes

- `.venv` is intentionally ignored in git. Recreate it per workspace.
- `.codex.gitconfig` is repo-local and used to avoid global `safe.directory` issues in sandboxed sessions.
- `.codex-tmp` is repo-local and used to avoid permission errors when sandbox users cannot write to `%LocalAppData%\\Temp`.
- `.platformio-local` is repo-local and used as `PLATFORMIO_CORE_DIR` to avoid home-directory ownership issues in sandboxed sessions.
- If `platformio` install fails in bootstrap, fix network/firewall access and rerun:

```powershell
.\.venv\Scripts\python.exe -m pip install platformio
pwsh -File scripts/preflight.ps1
```
