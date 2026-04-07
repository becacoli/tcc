"""
Conversões entre coordenadas do mundo e do planejador.

Single Responsibility:
- Funções para transformação de coordenadas entre espaços
- Mantém a abstração de mapa discretizado vs mundo contínuo
"""

from planning_context.context import PlanningContext


def world_to_planner_coords(x: float, y: float, context: PlanningContext) -> tuple:
    """
    Converte coordenadas do mundo para coordenadas do planejador (mapa discretizado).
    
    Mundo: Coordenadas reais (xmin..xmax, ymin..ymax)
    Planejador: Coordenadas discretizadas (0..width_px, 0..height_px)
    
    Args:
        x: Coordenada X no mundo
        y: Coordenada Y no mundo
        context: Contexto com informações de mapeamento
        
    Returns:
        tuple: (px, py) coordenadas no mapa do planejador
        
    Examples:
        >>> from planning_context import PlanningContext, WorldBounds
        >>> context = PlanningContext(
        ...     robot_pose=Pose(0, 0),
        ...     goal_pose=Pose(10, 10),
        ...     obstacles=[],
        ...     world_bounds=WorldBounds(0, 10, 0, 10),
        ...     map_size_pixels=(100, 100)
        ... )
        >>> world_to_planner_coords(5.0, 5.0, context)
        (50.0, 50.0)
    """
    map_w, map_h = context.map_size_pixels
    min_x, max_x, min_y, max_y = context.world_bounds.as_tuple
    
    # Evita divisão por zero
    if max_x <= min_x:
        px = 0.0
    else:
        px = (x - min_x) * map_w / (max_x - min_x)
    
    if max_y <= min_y:
        py = 0.0
    else:
        py = (y - min_y) * map_h / (max_y - min_y)
    
    return (px, py)


def planner_to_world_coords(px: float, py: float, context: PlanningContext) -> tuple:
    """
    Converte coordenadas do planejador (mapa) para coordenadas do mundo.
    
    Planejador: Coordenadas discretizadas (0..width_px, 0..height_px)
    Mundo: Coordenadas reais (xmin..xmax, ymin..ymax)
    
    Args:
        px: Coordenada X no mapa do planejador
        py: Coordenada Y no mapa do planejador
        context: Contexto com informações de mapeamento
        
    Returns:
        tuple: (x, y) coordenadas no mundo real
        
    Examples:
        >>> from planning_context import PlanningContext, WorldBounds
        >>> context = PlanningContext(
        ...     robot_pose=Pose(0, 0),
        ...     goal_pose=Pose(10, 10),
        ...     obstacles=[],
        ...     world_bounds=WorldBounds(0, 10, 0, 10),
        ...     map_size_pixels=(100, 100)
        ... )
        >>> planner_to_world_coords(50.0, 50.0, context)
        (5.0, 5.0)
    """
    map_w, map_h = context.map_size_pixels
    min_x, max_x, min_y, max_y = context.world_bounds.as_tuple
    
    # Evita divisão por zero
    if map_w > 0:
        x = min_x + (px / map_w) * (max_x - min_x)
    else:
        x = min_x
    
    if map_h > 0:
        y = min_y + (py / map_h) * (max_y - min_y)
    else:
        y = min_y
    
    return (x, y)
