[![Università di Brescia](https://img.shields.io/badge/Universit%C3%A0%20di%20Brescia-27326E?style=for-the-badge)](https://www.unibs.it/it)

# Robotica 2026 - Maze Solver project

![](/docs/res/project_preview.svg)

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
Follow the detailed setup instructions in [MMS Setup Guide](docs/mms.md) to install the Micromouse simulator and configure the algorithms.

## Documentation

**Setup and Usage:**
- **[MMS Setup Guide](docs/mms.md)** — Complete setup and usage guide for MMS simulator integration and run workflow
- **[Maze Format and Dataset Guide](mazes/README.md)** — Maze file format specification and maze-dataset conventions used by the project

**Project Components:**
- **[Source Package Overview](src/README.md)** — Package-level architecture: shared data structures, top-level modules, and subpackage map
- **[API Layer](src/api/README.md)** — Backend abstraction (`BaseAPI`) and concrete MMS/headless integrations (`MmsAPI`, `SimAPI`)
- **[Algorithms Module](src/algorithms/README.md)** — Exploration algorithms (`AStarExplorer`, `DStarLiteExplorer`) and shared planning loop in `BaseAlgorithm`
- **[Maze Parser Module](src/parser/README.md)** — Maze parsing pipeline from ASCII files to internal wall-matrix representation
- **[Metrics and Logging Module](src/metrics/README.md)** — Metrics collection, per-run logging lifecycle, and JSON export schema
- **[Tools and Utilities](tools/README.md)** — Supporting scripts for maze generation, connectivity filtering, and detour-index goal-placement inspection

**Maze Lookup:**
- [Online maze viewer](https://htmlpreview.github.io/?https://github.com/FrancescoGuarda/Rob-26-MazeSolver/blob/main/mazes/index.html) — browse and visualize the maze set in the browser

**Project Documentation:**
- **[Final Project Report](docs/Rob-26-MazeSolver_report.pdf)** — Final project report with experimental results and analysis

## Citations

This project builds upon the following works:

For citing this repository itself, use [CITATION.cff](CITATION.cff) (preferred by GitHub's citation tooling).

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

Suggested software citation for this repository:

> Guarda, F., & Moro, A. (2026). *Robotica 2026 - Maze Solver Project* [Software]. GitHub. <https://github.com/FrancescoGuarda/Rob-26-MazeSolver>

## License

[MIT License](LICENSE)