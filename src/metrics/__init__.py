"""
Metrics collection and export.

MetricsLogger records per-run performance statistics (moves, turns,
distinct cells visited, total visits, execution time) and saves them
to a timestamped JSON file in results/logs/.
"""
from src.metrics.logger import MetricsLogger

__all__ = ["MetricsLogger"]
