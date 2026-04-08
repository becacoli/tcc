"""
Extrator de dados do CoppeliaSim para PlanningContext.

Single Responsibility:
- Extrair dados do simulador CoppeliaSim
- Converter dados 3D para estruturas do PlanningContext
- Adapter Pattern: Adapta interface CoppeliaSim para PlanningContext
"""

from typing import Optional, List, Tuple

from planning_context.core import Pose, Obstacle, WorldBounds
from planning_context.context import PlanningContext, PlannerContextBuilder


def _transform_point(matrix, point):
    """Transforma um ponto local (x, y, z) para coordenadas mundo."""
    x, y, z = point
    return (
        matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
        matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
        matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
    )


def _cuboid_world_vertices_xy(sim, handle) -> list:
    """Retorna os 4 vértices XY do bbox local transformados para mundo."""
    min_x = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_x)
    min_y = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_y)
    max_x = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_x)
    max_y = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_y)

    matrix = sim.getObjectMatrix(handle, -1)
    world_pts = [
        _transform_point(matrix, (min_x, min_y, 0.0)),
        _transform_point(matrix, (max_x, min_y, 0.0)),
        _transform_point(matrix, (max_x, max_y, 0.0)),
        _transform_point(matrix, (min_x, max_y, 0.0)),
    ]
    return [(p[0], p[1]) for p in world_pts]


def _inflate_rect(bounds: Tuple[float, float, float, float], inflate_by: float) -> Tuple[float, float, float, float]:
    x_min, y_min, x_max, y_max = bounds
    if inflate_by <= 0.0:
        return bounds
    return (x_min - inflate_by, y_min - inflate_by, x_max + inflate_by, y_max + inflate_by)


def _clip_rect_to_world(
    bounds: Tuple[float, float, float, float],
    world_bounds: Optional[Tuple[float, float, float, float]],
    min_size: float = 1e-4,
) -> Optional[Tuple[float, float, float, float]]:
    """Recorta AABB para os limites do mundo e descarta retangulos degenerados."""
    x_min, y_min, x_max, y_max = bounds

    if world_bounds is not None:
        wx_min, wx_max, wy_min, wy_max = world_bounds
        x_min = max(x_min, wx_min)
        y_min = max(y_min, wy_min)
        x_max = min(x_max, wx_max)
        y_max = min(y_max, wy_max)

    if (x_max - x_min) < min_size or (y_max - y_min) < min_size:
        return None

    return (x_min, y_min, x_max, y_max)


def _rect_vertices_xy(bounds: Tuple[float, float, float, float]) -> List[Tuple[float, float]]:
    x_min, y_min, x_max, y_max = bounds
    return [
        (x_min, y_min),
        (x_max, y_min),
        (x_max, y_max),
        (x_min, y_max),
    ]


def _rect_from_handle_world(sim, handle) -> Tuple[tuple, float]:
    """
    Extrai retângulo (bounds + posição) de um objeto no CoppeliaSim.
    
    Args:
        sim: Objeto simulador
        handle: Handle do objeto CoppeliaSim
        
    Returns:
        tuple: ((x_min, y_min, x_max, y_max), yaw)
    """
    ori = sim.getObjectOrientation(handle, -1)
    yaw = ori[2]
    
    # Usa matriz do objeto para transformar os 4 cantos do bbox local.
    # Isso evita overestimation por dupla rotação em paredes (cuboid) rotacionadas.
    world_xy = _cuboid_world_vertices_xy(sim, handle)
    world_pts = [(x, y, 0.0) for (x, y) in world_xy]

    xs = [p[0] for p in world_pts]
    ys = [p[1] for p in world_pts]

    x_min = min(xs)
    x_max = max(xs)
    y_min = min(ys)
    y_max = max(ys)
    
    return ((x_min, y_min, x_max, y_max), yaw)


def _try_get_object_by_alias(sim, alias: str) -> int:
    """
    Tenta obter objeto por alias do CoppeliaSim.
    
    Args:
        sim: Objeto simulador
        alias: Nome do objeto
        
    Returns:
        int: Handle do objeto ou -1 se não encontrado
    """
    try:
        return sim.getObject(f"/{alias}")
    except Exception:
        return -1


def _normalize_alias(alias: str) -> str:
    """Normaliza nome de objeto do CoppeliaSim extraindo o último componente."""
    if not alias:
        return ""
    return alias.split("/")[-1]


def _wall_name_matches(alias: str, prefix: str) -> bool:
    """Verifica se nome de objeto começa com prefixo."""
    return _normalize_alias(alias).lower().startswith(prefix.lower())


class CoppeliaSimContextExtractor:
    """
    Extrai dados do CoppeliaSim e cria um PlanningContext independente.
    
    Strategy Pattern: Encapsula a lógica de extração do CoppeliaSim.
    
    Fluxo típico:
        1. Conectar ao CoppeliaSim
        2. Chamar extract_context() para pegar estado atual
        3. Usar context para planejamento (isolado do simulador)
        4. Repetir conforme necessário (loop recorrente)
    
    Args:
        sim: Objeto simulador do CoppeliaSim
        robot_path: Caminho do objeto robô (ex: "/PioneerP3DX")
        goal_object: Caminho do objeto objetivo (ex: "/GoalConfiguration")
        walls_prefix: Prefixo dos nomes de obstáculos (ex: "wall_")
        floor_alias: Nome do objeto chão (para auto-detectar limites)
    """
    
    def __init__(self, sim, robot_path: str = "/PioneerP3DX",
                 goal_object: Optional[str] = None,
                 walls_prefix: str = "wall_",
                 floor_alias: str = "floor"):
        self.sim = sim
        self.robot_path = robot_path
        self.goal_object = goal_object
        self.walls_prefix = walls_prefix
        self.floor_alias = floor_alias
    
    def extract_context(self,
                       world_bounds: Optional[Tuple[float, float, float, float]] = None,
                       map_size_pixels: Tuple[int, int] = (100, 100),
                       wall_inflate: float = 0.0,
                       cuboid_only: bool = True,
                       obstacle_primitives: Optional[str] = None,
                       allow_start_in_obstacle: bool = False,
                       scenario_name: str = "scene") -> PlanningContext:
        """
        Extrai estado atual do simulador e cria PlanningContext.
        
        Args:
            world_bounds: (min_x, max_x, min_y, max_y) ou None para auto-detectar
            map_size_pixels: Tamanho do mapa para planejamento
            wall_inflate: Inflação dos obstáculos (margem de segurança)
            cuboid_only: Se True, usa apenas obstáculos com primitive shape cuboid
            obstacle_primitives: Filtro de primitivas: "cuboid", "cuboid_spheroid" ou "all"
            allow_start_in_obstacle: Se True, não falha quando start estiver em obstáculo
            scenario_name: Nome do cenário para metadata
        
        Returns:
            PlanningContext pronto para planejamento e isolado do simulador
            
        Raises:
            RuntimeError: Se robô ou objetivo não forem encontrados
        """
        
        # ===== 1. Extrai pose do robô =====
        robot_pose = self._extract_robot_pose()
        print(f"✓ Robot pose: {robot_pose}")
        
        # ===== 2. Extrai pose do objetivo =====
        goal_pose = self._extract_goal_pose()
        print(f"✓ Goal pose: {goal_pose}")
        
        # ===== 3. Auto-detecta limites do mundo se necessário =====
        if world_bounds is None:
            world_bounds = self._auto_detect_world_bounds()
        
        world_bounds_obj = WorldBounds(*world_bounds)
        print(f"✓ World bounds: {world_bounds_obj}")

        primitive_mode = obstacle_primitives
        if primitive_mode is None:
            primitive_mode = "cuboid" if cuboid_only else "all"
        
        # ===== 4. Extrai obstáculos (paredes) =====
        obstacles, obstacle_details = self._extract_obstacles(
            wall_inflate,
            world_bounds=world_bounds,
            cuboid_only=cuboid_only,
            obstacle_primitives=primitive_mode,
        )
        print(f"✓ Extraídos {len(obstacles)} obstáculos")
        
        # ===== 5. Constrói e valida contexto =====
        builder = PlannerContextBuilder()
        builder.set_robot_pose(robot_pose.x, robot_pose.y, robot_pose.theta)
        builder.set_goal_pose(goal_pose.x, goal_pose.y, goal_pose.theta)
        builder.add_obstacles(obstacles)
        builder.set_world_bounds(*world_bounds)
        builder.set_map_size_pixels(*map_size_pixels)
        builder.set_metadata("scenario", scenario_name)
        builder.set_metadata("extracted_from", "coppelia_sim")
        builder.set_metadata("cuboid_only", cuboid_only)
        builder.set_metadata("obstacle_primitives", primitive_mode)
        builder.set_metadata("allow_start_in_obstacle", allow_start_in_obstacle)
        builder.set_metadata("obstacle_details", obstacle_details)
        
        context = builder.build()
        print(f"✓ Contexto criado e validado\n")
        
        return context
    
    def _extract_robot_pose(self) -> Pose:
        """Extrai pose do objeto robô do simulador."""
        try:
            robot = self.sim.getObject(self.robot_path)
        except Exception as e:
            raise RuntimeError(f"Robô '{self.robot_path}' não encontrado: {e}")
        
        pos = self.sim.getObjectPosition(robot, -1)
        ori = self.sim.getObjectOrientation(robot, -1)
        
        return Pose(x=pos[0], y=pos[1], theta=ori[2])
    
    def _extract_goal_pose(self) -> Pose:
        """Extrai pose do objeto objetivo do simulador."""
        if self.goal_object is None:
            raise RuntimeError("Objetivo não foi especificado")
        
        try:
            goal = self.sim.getObject(self.goal_object)
        except Exception as e:
            raise RuntimeError(f"Objetivo '{self.goal_object}' não encontrado: {e}")
        
        pos = self.sim.getObjectPosition(goal, -1)
        ori = self.sim.getObjectOrientation(goal, -1)
        
        return Pose(x=pos[0], y=pos[1], theta=ori[2])
    
    def _auto_detect_world_bounds(self) -> Tuple[float, float, float, float]:
        """Auto-detecta limites do mundo baseado no objeto chão (floor)."""
        floor_handle = _try_get_object_by_alias(self.sim, self.floor_alias)
        
        if floor_handle == -1:
            # Fallback: limites padrão
            print(f"⚠ Chão '{self.floor_alias}' não encontrado, usando limites padrão")
            return (-5.0, 5.0, -5.0, 5.0)
        
        bounds, _ = _rect_from_handle_world(self.sim, floor_handle)
        x_min, y_min, x_max, y_max = bounds
        
        return (x_min, x_max, y_min, y_max)
    
    def _is_allowed_shape(self, handle, obstacle_primitives: str) -> bool:
        """Verifica via API se shape atende ao filtro de primitivas."""
        if obstacle_primitives == "all":
            return True

        try:
            primitive_type = self.sim.getObjectInt32Param(handle, self.sim.shapeintparam_primitive_type)

            allowed = {self.sim.primitiveshape_cuboid}
            if obstacle_primitives == "cuboid_spheroid":
                spheroid_type = getattr(self.sim, "primitiveshape_spheroid", None)
                if spheroid_type is not None:
                    allowed.add(spheroid_type)

            return primitive_type in allowed
        except Exception:
            # Fallback para compatibilidade entre versões/API bindings
            return True

    def _extract_obstacles(
        self,
        inflate_by: float = 0.0,
        world_bounds: Optional[Tuple[float, float, float, float]] = None,
        cuboid_only: bool = True,
        obstacle_primitives: str = "cuboid",
    ) -> Tuple[List[Obstacle], List[dict]]:
        """
        Extrai lista de obstáculos (paredes) do simulador.
        
        Args:
            inflate_by: Margem de segurança para adicionar aos obstáculos
            
        Returns:
            Lista de objetos Obstacle
        """
        obstacles = []
        obstacle_details = []
        
        try:
            shape_handles = self.sim.getObjectsInTree(
                self.sim.handle_scene,
                self.sim.object_shape_type,
                0
            )
        except Exception:
            print("⚠ Erro ao extrair shapes, retornando lista vazia de obstáculos")
            return [], []
        
        for handle in shape_handles:
            try:
                alias = self.sim.getObjectAlias(handle)
            except Exception:
                continue
            
            # Filtra por prefixo de parede
            if not _wall_name_matches(alias, self.walls_prefix):
                continue

            # Filtro de primitivas (cuboid, cuboid+spheroid, all)
            if cuboid_only and (not self._is_allowed_shape(handle, obstacle_primitives)):
                continue
            
            try:
                (x_min, y_min, x_max, y_max), yaw = _rect_from_handle_world(self.sim, handle)

                raw_bounds = (x_min, y_min, x_max, y_max)
                planner_bounds = _inflate_rect(raw_bounds, inflate_by)
                planner_bounds = _clip_rect_to_world(planner_bounds, world_bounds)
                if planner_bounds is None:
                    continue

                x_min_p, y_min_p, x_max_p, y_max_p = planner_bounds
                
                cx = (x_min_p + x_max_p) / 2.0
                cy = (y_min_p + y_max_p) / 2.0
                width = abs(x_max_p - x_min_p)
                height = abs(y_max_p - y_min_p)
                
                obs = Obstacle(
                    cx=cx, cy=cy,
                    width=width, height=height,
                    name=_normalize_alias(alias),
                    # _rect_from_handle_world ja retorna um retangulo alinhado aos eixos
                    # (AABB no mundo). Portanto, nao aplicamos rotacao novamente aqui.
                    theta=0.0
                )
                obstacles.append(obs)

                world_vertices = _cuboid_world_vertices_xy(self.sim, handle)
                planner_vertices = _rect_vertices_xy(planner_bounds)
                obstacle_details.append({
                    "name": _normalize_alias(alias),
                    "center": {"x": cx, "y": cy},
                    "geometry": {"width": width, "height": height},
                    "theta": yaw,
                    "vertices_xy": world_vertices,
                    "planner_bbox_vertices_xy": planner_vertices,
                    "raw_bounds": {
                        "x_min": raw_bounds[0],
                        "y_min": raw_bounds[1],
                        "x_max": raw_bounds[2],
                        "y_max": raw_bounds[3],
                    },
                    "bounds": {
                        "x_min": x_min_p,
                        "y_min": y_min_p,
                        "x_max": x_max_p,
                        "y_max": y_max_p,
                    },
                })
                
            except Exception as e:
                print(f"⚠ Erro ao processar obstáculo {alias}: {e}")
                continue
        
        return obstacles, obstacle_details


def create_context_from_coppelia(sim, robot_path: str = "/PioneerP3DX",
                                 goal_object: str = "/GoalConfiguration",
                                 world_bounds: Optional[Tuple[float, float, float, float]] = None,
                                 map_size_pixels: Tuple[int, int] = (100, 100),
                                 wall_inflate: float = 0.0,
                                 cuboid_only: bool = True,
                                 obstacle_primitives: Optional[str] = None,
                                 allow_start_in_obstacle: bool = False) -> PlanningContext:
    """
    Função auxiliar para extrair contexto do CoppeliaSim em uma única chamada.
    
    Args:
        sim: Objeto simulador
        robot_path: Caminho do robô
        goal_object: Caminho do objetivo
        world_bounds: Limites do mundo ou None para auto-detectar
        map_size_pixels: Tamanho do mapa de planejamento
        wall_inflate: Margem de segurança para obstáculos
        cuboid_only: Se True, mantém apenas obstáculos primitive cuboid
        obstacle_primitives: Filtro de primitivas: "cuboid", "cuboid_spheroid" ou "all"
        allow_start_in_obstacle: Se True, permite start inicialmente em obstáculo
        
    Returns:
        PlanningContext pronto para planejamento
        
    Examples:
        >>> from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        >>> client = RemoteAPIClient("localhost", 23000)
        >>> sim = client.getObject("sim")
        >>> context = create_context_from_coppelia(
        ...     sim=sim,
        ...     robot_path="/PioneerP3DX",
        ...     goal_object="/GoalConfiguration",
        ...     wall_inflate=0.26
        ... )
    """
    extractor = CoppeliaSimContextExtractor(
        sim=sim,
        robot_path=robot_path,
        goal_object=goal_object,
    )
    return extractor.extract_context(
        world_bounds=world_bounds,
        map_size_pixels=map_size_pixels,
        wall_inflate=wall_inflate,
        cuboid_only=cuboid_only,
        obstacle_primitives=obstacle_primitives,
        allow_start_in_obstacle=allow_start_in_obstacle,
    )
