"""
Planning Context - Isolamento independente do planejador de caminho

Módulo que centraliza todos os dados necessários para planejamento,
desacoplando o planner de outras partes da pilha de navegação
(controle, estimação, mapeamento).

Componentes principais:
    - core: Tipos básicos (Pose, Obstacle, WorldBounds)
    - context: PlanningContext e builder
    - coordinator: Conversões de coordenadas
    - extractors: Adaptadores para diferentes fontes (CoppeliaSim, etc.)

Uso típico:
    >> from planning_context import PlannerContextBuilder, PlanningContext
    >> from planning_context import world_to_planner_coords
    >>
    >> # Criar contexto
    >> context = (PlannerContextBuilder()
    ...     .set_robot_pose(1.0, 2.0)
    ...     .set_goal_pose(9.0, 8.0)
    ...     .set_world_bounds(0.0, 10.0, 0.0, 10.0)
    ...     .add_obstacle(5.0, 5.0, 2.0, 2.0)
    ...     .build())
    >>
    >> # Converter coordenadas
    >> planner_coords = world_to_planner_coords(5.0, 5.0, context)
    >>
    >> # Usar com algoritmos de planejamento
    >> start = world_to_planner_coords(*context.robot_position(), context)
    >> goal = world_to_planner_coords(*context.goal_position(), context)
    >> obstacles = context.get_obstacles_as_rects()
"""

# Tipos básicos
from planning_context.core import Pose, Obstacle, WorldBounds

# Contexto e builder
from planning_context.context import PlanningContext, PlannerContextBuilder

# Conversões de coordenadas
from planning_context.coordinator import (
    world_to_planner_coords,
    planner_to_world_coords,
)

# Extractors
from planning_context.extractors import (
    CoppeliaSimContextExtractor,
    create_context_from_coppelia,
)

__all__ = [
    # Core types
    "Pose",
    "Obstacle",
    "WorldBounds",
    
    # Context
    "PlanningContext",
    "PlannerContextBuilder",
    
    # Coordinate conversions
    "world_to_planner_coords",
    "planner_to_world_coords",
    
    # Extractors
    "CoppeliaSimContextExtractor",
    "create_context_from_coppelia",
]

__version__ = "1.0.0"
__author__ = "TCC - Path Planning"
