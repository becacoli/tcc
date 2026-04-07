"""
Tipos de dados básicos para o contexto de planejamento.

Single Responsibility:
- Pose: Representa posição e orientação
- Obstacle: Representa um obstáculo no mundo
- WorldBounds: Limites do espaço de trabalho
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class Pose:
    """
    Representa a pose de um objeto: posição (x, y) e orientação θ (em radianos).
    
    Attributes:
        x: Coordenada X
        y: Coordenada Y
        theta: Orientação em radianos (padrão: 0.0)
    
    Examples:
        >>> robot = Pose(1.5, 2.3, 0.0)
        >>> robot.position
        (1.5, 2.3)
    """
    x: float
    y: float
    theta: float = 0.0  # radianos
    
    @property
    def position(self) -> Tuple[float, float]:
        """Retorna apenas (x, y)."""
        return (self.x, self.y)
    
    @property
    def as_tuple(self) -> Tuple[float, float, float]:
        """Retorna (x, y, θ)."""
        return (self.x, self.y, self.theta)
    
    def __repr__(self) -> str:
        return f"Pose(x={self.x:.2f}, y={self.y:.2f}, θ={math.degrees(self.theta):.1f}°)"


@dataclass
class Obstacle:
    """
    Representa um obstáculo no mundo com centro e geometria.
    
    Attributes:
        cx: Centro X
        cy: Centro Y
        width: Largura (tamanho em X)
        height: Altura (tamanho em Y)
        name: Nome do obstáculo (padrão: "obstacle")
        theta: Rotação em radianos (padrão: 0.0)
    
    Examples:
        >>> wall = Obstacle(cx=5.0, cy=5.0, width=2.0, height=0.5, name="wall_1")
        >>> wall.center
        (5.0, 5.0)
        >>> wall.bounds
        (4.0, 4.75, 6.0, 5.25)
    """
    cx: float        # centro x
    cy: float        # centro y
    width: float     # tamanho em x
    height: float    # tamanho em y
    name: str = "obstacle"
    theta: float = 0.0  # rotação em radianos
    
    @property
    def center(self) -> Tuple[float, float]:
        """Retorna o centro do obstáculo (cx, cy)."""
        return (self.cx, self.cy)
    
    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        """
        Retorna (x_min, y_min, x_max, y_max) - bounding box alinhado em eixo X/Y.
        
        Aviso: Se theta != 0, os bounds calculados são aproximados (AABB).
        Para colisões precisas com rotação, considere a rotação separadamente.
        """
        half_w = self.width / 2.0
        half_h = self.height / 2.0
        
        # Se rotacionado, bounding box é maior (aproximação conservadora)
        if self.theta != 0:
            # Calcula a diagonal e usa como raio para AABB
            diagonal = math.hypot(half_w, half_h)
            return (
                self.cx - diagonal,
                self.cy - diagonal,
                self.cx + diagonal,
                self.cy + diagonal,
            )
        else:
            return (
                self.cx - half_w,
                self.cy - half_h,
                self.cx + half_w,
                self.cy + half_h,
            )
    
    def __repr__(self) -> str:
        return (f"Obstacle({self.name}, center=({self.cx:.2f}, {self.cy:.2f}), "
                f"size=({self.width:.2f}x{self.height:.2f}))")


@dataclass
class WorldBounds:
    """
    Define os limites do espaço de trabalho e tamanho do mapa de planejamento.
    
    Attributes:
        min_x: Limite mínimo em X
        max_x: Limite máximo em X
        min_y: Limite mínimo em Y
        max_y: Limite máximo em Y
    
    Examples:
        >>> world = WorldBounds(0.0, 10.0, 0.0, 10.0)
        >>> world.size
        (10.0, 10.0)
    """
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    
    @property
    def size(self) -> Tuple[float, float]:
        """Retorna (largura, altura) do mundo em coordenadas reais."""
        return (self.max_x - self.min_x, self.max_y - self.min_y)
    
    @property
    def as_tuple(self) -> Tuple[float, float, float, float]:
        """Retorna (min_x, max_x, min_y, max_y)."""
        return (self.min_x, self.max_x, self.min_y, self.max_y)
    
    def __repr__(self) -> str:
        return (f"WorldBounds(x=[{self.min_x:.2f}, {self.max_x:.2f}], "
                f"y=[{self.min_y:.2f}, {self.max_y:.2f}])")
