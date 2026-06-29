# Scripts

This directory contains utility scripts for running and managing processes.

### Setup

Before running any script, make it executable:

```bash
chmod +x scripts/script_name.sh
```

Or make all scripts executable at once:

```bash
chmod +x scripts/*.sh
```

---

### Available Commands

| Command | Flags | Description |
|---------|-------|-------------|
| `./scripts/clean.sh` | None | Clean Python cache files and directories (`__pycache__`, `.pytest_cache`, `.pyc`, `.pyo`, `.coverage`). |

---

### Modify Settings 

To modify settings, edit directly the parameters in the respective `.sh` file.