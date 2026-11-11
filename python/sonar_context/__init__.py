"""Sonar Context: imaging-sonar place recognition (ICRA 2023)."""
from .descriptor import as_range_beam, polar_key, sonar_context
from .manager import Loop, SonarContextManager
from .matching import column_distance, column_similarity_matrix, row_distance

__version__ = "2.0.1"
__all__ = ["sonar_context", "polar_key", "as_range_beam", "column_distance", "row_distance",
           "column_similarity_matrix", "SonarContextManager", "Loop", "__version__"]
