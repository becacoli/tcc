"""
Contexto de planejamento e seu builder.

Single Responsibility:
- PlanningContext: Interface clara com todos os dados para planejamento
- PlannerContextBuilder: Constrói contexto de forma fluente (Factory pattern)
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

from planning_context.core import Pose, Obstacle, WorldBounds


@dataclass
class PlanningContext:
    """
    Contexto completo e independente para planejamento de caminho.
    
    Centraliza todos os dados necessários para executar um algoritmo de planejamento
    sem dependências de outras partes da pilha (controle, mapeamento, estimação).
    
    Attributes:
        robot_pose: Pose atual do robô (xr, yr, θr)
        goal_pose: Pose desejada do objetivo (xg, yg, θg)
        obstacles: Lista de obstáculos com centros e geometria
        world_bounds: Limites e tamanho do mapa
        map_size_pixels: Tamanho do mapa discretizado (largura, altura)
        metadata: Dicionário com informações extras (cenário, configuração, etc.)
    
    Examples:
        >>> context = PlanningContext(
        ...     robot_pose=Pose(1.0, 1.0),
        ...     goal_pose=Pose(9.0, 9.0),
        ...     obstacles=[],
        ...     world_bounds=WorldBounds(0, 10, 0, 10),
        ...     map_size_pixels=(100, 100)
        ... )
        >>> context.robot_position()
        (1.0, 1.0)
    """
    
    robot_pose: Pose
    goal_pose: Pose
    obstacles: List[Obstacle]
    world_bounds: WorldBounds
    map_size_pixels: tuple = (100, 100)
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def robot_position(self) -> tuple:
        """Retorna posição atual do robô (xr, yr)."""
        return self.robot_pose.position
    
    def goal_position(self) -> tuple:
        """Retorna posição do objetivo (xg, yg)."""
        return self.goal_pose.position
    
    def robot_pose_full(self) -> tuple:
        """Retorna pose completa do robô (xr, yr, θr)."""
        return self.robot_pose.as_tuple
    
    def goal_pose_full(self) -> tuple:
        """Retorna pose completa do objetivo (xg, yg, θg)."""
        return self.goal_pose.as_tuple
    
    def get_obstacles_as_rects(self) -> List[tuple]:
        """
        Retorna lista de obstáculos como retângulos (x_min, y_min, x_max, y_max).
        
        Compatível com algoritmos que usam geometria retangular.
        """
        return [obs.bounds for obs in self.obstacles]
    
    def get_obstacle_by_name(self, name: str) -> Optional[Obstacle]:
        """Retorna um obstáculo específico pelo nome."""
        return next((obs for obs in self.obstacles if obs.name == name), None)

    def to_snapshot(self) -> Dict[str, Any]:
        """
        Retorna um snapshot explícito e serializável para avaliação do planner.

        Estrutura:
            - robot_pose: (xr, yr, theta_r)
            - goal_pose: (xg, yg, theta_g)
            - obstacles: centros e geometria
        """
        return {
            "robot_pose": {
                "xr": self.robot_pose.x,
                "yr": self.robot_pose.y,
                "theta_r": self.robot_pose.theta,
            },
            "goal_pose": {
                "xg": self.goal_pose.x,
                "yg": self.goal_pose.y,
                "theta_g": self.goal_pose.theta,
            },
            "obstacles": [
                {
                    "name": obs.name,
                    "cx": obs.cx,
                    "cy": obs.cy,
                    "width": obs.width,
                    "height": obs.height,
                    "theta": obs.theta,
                    "bounds": {
                        "x_min": obs.bounds[0],
                        "y_min": obs.bounds[1],
                        "x_max": obs.bounds[2],
                        "y_max": obs.bounds[3],
                    },
                }
                for obs in self.obstacles
            ],
        }
    
    def validate(self) -> bool:
        """
        Valida se o contexto está bem formado.
        
        Retorna:
            True se válido
            
        Raises:
            ValueError: Se inválido (robot ou goal fora dos limites, colisão inicial)
        """
        # Verifica se robot está dentro do mundo
        if not self._point_in_bounds(self.robot_pose.position):
            raise ValueError(f"Robot pose {self.robot_pose} está fora dos limites do mundo")
        
        # Verifica se goal está dentro do mundo
        if not self._point_in_bounds(self.goal_pose.position):
            raise ValueError(f"Goal pose {self.goal_pose} está fora dos limites do mundo")
        
        # Verifica se robot não está em colisão inicial
        colliding_obstacle = self._robot_colliding_obstacle()
        allow_start_in_obstacle = bool(self.metadata.get("allow_start_in_obstacle", False))
        if colliding_obstacle is not None and not allow_start_in_obstacle:
            raise ValueError(
                f"Robot começando dentro de um obstáculo! ({colliding_obstacle.name})"
            )
        
        return True
    
    def _point_in_bounds(self, point: tuple) -> bool:
        """Verifica se um ponto está dentro dos limites do mundo."""
        x, y = point
        return (self.world_bounds.min_x <= x <= self.world_bounds.max_x and
                self.world_bounds.min_y <= y <= self.world_bounds.max_y)
    
    def _robot_colliding_obstacle(self) -> Optional[Obstacle]:
        """Verifica se robot está dentro de um obstáculo.
        
        Permite contato com borda (ou penetração mínima numérica) e só
        considera colisão quando houver penetração significativa.
        """
        rx, ry = self.robot_pose.position
        penetration_margin = 0.03  # 3cm: tolera contato de borda
        
        for obs in self.obstacles:
            x_min, y_min, x_max, y_max = obs.bounds

            inside_x = x_min <= rx <= x_max
            inside_y = y_min <= ry <= y_max
            if not (inside_x and inside_y):
                continue

            # Profundidade de penetração até a borda mais próxima.
            # Se muito pequena, tratamos como contato/erro numérico.
            depth_x = min(rx - x_min, x_max - rx)
            depth_y = min(ry - y_min, y_max - ry)
            penetration = min(depth_x, depth_y)
            if penetration > penetration_margin:
                return obs
        
        return None
    
    def __repr__(self) -> str:
        return (
            f"PlanningContext(\n"
            f"  robot: {self.robot_pose}\n"
            f"  goal:  {self.goal_pose}\n"
            f"  obstacles: {len(self.obstacles)}\n"
            f"  world: {self.world_bounds}\n"
            f")"
        )


class PlannerContextBuilder:
    """
    Builder para construir PlanningContext de forma fluente (Fluent Interface).
    
    Factory Pattern: Facilita construção de objetos complexos.
    
    Examples:
        >>> context = (PlannerContextBuilder()
        ...     .set_robot_pose(1.0, 2.0)
        ...     .set_goal_pose(9.0, 8.0)
        ...     .set_world_bounds(0.0, 10.0, 0.0, 10.0)
        ...     .set_map_size_pixels(100, 100)
        ...     .add_obstacle(5.0, 5.0, 2.0, 2.0, "wall_1")
        ...     .build())
    """
    
    def __init__(self):
        self._robot_pose: Optional[Pose] = None
        self._goal_pose: Optional[Pose] = None
        self._obstacles: List[Obstacle] = []
        self._world_bounds: Optional[WorldBounds] = None
        self._map_size_pixels: tuple = (100, 100)
        self._metadata: Dict[str, Any] = {}
    
    def set_robot_pose(self, x: float, y: float, theta: float = 0.0) -> "PlannerContextBuilder":
        """Define pose do robô. Retorna self para encadeamento."""
        self._robot_pose = Pose(x, y, theta)
        return self
    
    def set_goal_pose(self, x: float, y: float, theta: float = 0.0) -> "PlannerContextBuilder":
        """Define pose do objetivo. Retorna self para encadeamento."""
        self._goal_pose = Pose(x, y, theta)
        return self
    
    def add_obstacle(self, cx: float, cy: float, width: float, height: float,
                     name: str = None, theta: float = 0.0) -> "PlannerContextBuilder":
        """Adiciona um obstáculo. Retorna self para encadeamento."""
        if name is None:
            name = f"obstacle_{len(self._obstacles)}"
        self._obstacles.append(Obstacle(cx, cy, width, height, name, theta))
        return self
    
    def add_obstacles(self, obstacles: List[Obstacle]) -> "PlannerContextBuilder":
        """Adiciona lista de obstáculos. Retorna self para encadeamento."""
        self._obstacles.extend(obstacles)
        return self
    
    def set_world_bounds(self, min_x: float, max_x: float,
                        min_y: float, max_y: float) -> "PlannerContextBuilder":
        """Define limites do mundo. Retorna self para encadeamento."""
        self._world_bounds = WorldBounds(min_x, max_x, min_y, max_y)
        return self
    
    def set_map_size_pixels(self, width: int, height: int) -> "PlannerContextBuilder":
        """Define tamanho do mapa em pixels. Retorna self para encadeamento."""
        self._map_size_pixels = (width, height)
        return self
    
    def set_metadata(self, key: str, value: Any) -> "PlannerContextBuilder":
        """Adiciona informação de metadata. Retorna self para encadeamento."""
        self._metadata[key] = value
        return self
    
    def build(self) -> PlanningContext:
        """
        Constrói o PlanningContext validando todos os campos necessários.
        
        Returns:
            PlanningContext validado e pronto para uso
            
        Raises:
            ValueError: Se algum campo obrigatório não foi definido
        """
        if self._robot_pose is None:
            raise ValueError("Robot pose não foi definida")
        if self._goal_pose is None:
            raise ValueError("Goal pose não foi definida")
        if self._world_bounds is None:
            raise ValueError("World bounds não foram definidas")
        
        context = PlanningContext(
            robot_pose=self._robot_pose,
            goal_pose=self._goal_pose,
            obstacles=self._obstacles,
            world_bounds=self._world_bounds,
            map_size_pixels=self._map_size_pixels,
            metadata=self._metadata,
        )
        context.validate()
        return context
