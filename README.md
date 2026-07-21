# Robotica 2026 - Maze Solver project

## Project Overview

## Team Members

- **Guarda Francesco** - matricola 749674 - f.guarda@studenti.unibs.it
- **Moro Andrea** - matricola 749183 - a.moro003@studenti.unibs.it

## Setup Instructions

### Prerequisites

- Python 3.8 or higher
- Git

### Installation

1. Clone the repository:
```bash
git clone https://github.com/FrancescoGuarda/Rob-26-MazeSolver.git
cd Rob-26-MazeSolver
```

2. Create and activate a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

See [requirements.txt](requirements.txt) for the full list of dependencies.

4. Download and configure the MMS simulator:
Follow the detailed setup instructions in [mms.md](docs/mms.md) to install the Micromouse simulator and configure the algorithms.

## Documentation

**Setup and Usage:**
- **[mms.md](docs/mms.md)** — Complete setup and usage guide for the MMS simulator integration
- **[mazes/README.md](mazes/README.md)** — Maze file format specifications; all mazes live in [`mazes/txt/`](mazes/txt/), with exploration difficulty  determined by goal placement (see `tools/README.md`)
- **[src/api/README.md](src/api/README.md)** — API interface documentation for `mms_api` and `sim_api`

**Project Components:**
- **[src/algorithms/README.md](src/algorithms/README.md)** — `AStarExplorer` and `DStarLiteExplorer` internals, shared `BaseAlgorithm`, GUI/stderr diagnostics
- **[src/parser/README.md](src/parser/README.md)** — ASCII maze file parsing
- **[src/metrics/README.md](src/metrics/README.md)** — `MetricsLogger` and the exported JSON log schema
- **[tools/README.md](tools/README.md)** — maze generation, connectivity filtering, and the goal-placement/detour-index CLI

**Maze Lookup:**
- [Online maze viewer](https://htmlpreview.github.io/?https://github.com/FrancescoGuarda/Rob-26-MazeSolver/blob/main/mazes/index.html) — browse and visualize the maze set in the browser

**Project Documentation:**
- **[docs/Rob_26_proposal.md](docs/Rob_26_proposal.md)** — Original project proposal and objectives
- **[docs/notes.md](docs/notes.md)** — Development notes, design decisions, and algorithm specifications
- **[docs/Rob-26-MazeSolver_report.md](docs/Rob-26-MazeSolver_report.md)** — Final project report with experimental results and analysis

## Citations

This project builds upon the following works:

- [1] mackorone, *mms — A Micromouse simulator*, version 1.2.0, software (MIT License), GitHub repository, 2024. [Online]. Available: <https://github.com/mackorone/mms> (accessed Jul. 17, 2026). — Used to run and visualize the maze-solving algorithms.
- [2] J. Weisberg, *Micromouse Maze Collection*, dataset. [Online]. Available: <https://www.tcp4me.com/mmr/mazes/> (accessed Jul. 17, 2026). — Base collection of standard competition mazes, extended for this project's test set.

BibTeX entries are also available in [CITATIONS.bib](CITATIONS.bib):

```bibtex
@misc{mms,
  author       = {{mackorone}},
  title        = {mms --- A Micromouse Simulator},
  year         = {2024},
  version      = {1.2.0},
  howpublished = {\url{https://github.com/mackorone/mms}},
  note         = {MIT License. Accessed: 2026-07-17}
}

@misc{weisberg_mazes,
  author       = {Weisberg, Jeff},
  title        = {Micromouse Maze Collection},
  howpublished = {\url{https://www.tcp4me.com/mmr/mazes/}},
  note         = {Accessed: 2026-07-17}
}
```

## License

[MIT License](LICENSE)