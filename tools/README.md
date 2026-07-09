# Tools

This directory contains standalone Python scripts for creating and curating maze files.

---

### Available Scripts

| Script | Description |
|--------|-------------|
| `tools/gen_maze.py` | Generate a perfect maze (no loops, no islands) in the mms ASCII text format used by this project. |
| `tools/filter_connected.py` | Scan a directory of mazes and delete any that aren't fully connected or fail to parse. |

---

### Usage

```bash
python3 tools/gen_maze.py --rows 16 --cols 16 --seed 1 -o out.txt
python3 tools/filter_connected.py --dir mazes/txt --dry-run
```

See each script's module docstring for full options.
