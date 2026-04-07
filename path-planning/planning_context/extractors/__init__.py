"""
Extractors para PlanningContext

Exporta adaptadores que extraem dados de diferentes fontes
e convertem para PlanningContext.
"""

from planning_context.extractors.coppelia import (
    CoppeliaSimContextExtractor,
    create_context_from_coppelia,
)

__all__ = [
    "CoppeliaSimContextExtractor",
    "create_context_from_coppelia",
]
