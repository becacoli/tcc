# Planning Context - Módulo de Isolamento do Planejador

Estrutura organizada de forma limpa seguindo princípios de Clean Code e SOLID.

## 📁 Estrutura do Módulo

```
planning_context/
├── __init__.py                 ← Exporta interface pública (única entrada)
├── core.py                     ← Single Responsibility: tipos básicos
│   ├── Pose
│   ├── Obstacle
│   └── WorldBounds
├── context.py                  ← Single Responsibility: contexto + builder
│   ├── PlanningContext
│   └── PlannerContextBuilder
├── coordinator.py              ← Single Responsibility: conversões
│   ├── world_to_planner_coords()
│   └── planner_to_world_coords()
└── extractors/                 ← Adaptadores para diferentes fontes
    ├── __init__.py
    └── coppelia.py             ← Extrator do CoppeliaSim
        ├── CoppeliaSimContextExtractor
        └── create_context_from_coppelia()
```

## 🎯 Princípios Aplicados

### 1. **Single Responsibility Principle (SRP)**
- Cada arquivo tem uma única razão para mudar
- `core.py`: Apenas tipos de dados
- `context.py`: Apenas lógica de contexto
- `coordinator.py`: Apenas conversões
- `extractors/`: Apenas adaptadores

### 2. **Dependency Inversion Principle (DIP)**
- Módulos dependem de abstrações, não de implementações concretas
- `coordinator.py` depende de `PlanningContext` (abstração)
- `extractors/coppelia.py` depende de `PlannerContextBuilder` (interface)

### 3. **Open/Closed Principle (OCP)**
- Aberto para extensão, fechado para modificação
- Fácil adicionar novos extractors (ex: `extractors/ros.py`, `extractors/unity.py`)
- Sem modificar código existente

### 4. **Factory Pattern**
- `PlannerContextBuilder`: Cria `PlanningContext` de forma robusta
- Validação centralizada no `build()`

### 5. **Adapter Pattern**
- `CoppeliaSimContextExtractor`: Adapta interface do CoppeliaSim
- Isola código específico do simulador

## 📦 Como Usar

### Importação Única (Interface Pública)

```python
# Sempre importar de planning_context, nunca de submodulos
from planning_context import (
    Pose,
    Obstacle,
    WorldBounds,
    PlanningContext,
    PlannerContextBuilder,
    world_to_planner_coords,
    planner_to_world_coords,
    CoppeliaSimContextExtractor,
    create_context_from_coppelia,
)
```

### Evitar (Detalhes de Implementação)

```python
# ❌ NÃO fazer isto (quebra encapsulamento)
from planning_context.core import Pose
from planning_context.context import PlanningContext
from planning_context.coordinator import world_to_planner_coords
from planning_context.extractors.coppelia import CoppeliaSimContextExtractor
```

## 🔧 Extensão do Módulo

### Adicionar novo Extrator (ex: ROS)

1. Criar arquivo `planning_context/extractors/ros.py`:

```python
# planning_context/extractors/ros.py
from planning_context.context import PlannerContextBuilder

class RosContextExtractor:
    def extract_context(self, ...):
        builder = PlannerContextBuilder()
        # ... populate builder ...
        return builder.build()

def create_context_from_ros(...):
    extractor = RosContextExtractor()
    return extractor.extract_context(...)
```

2. Atualizar `planning_context/extractors/__init__.py`:

```python
from planning_context.extractors.ros import (
    RosContextExtractor,
    create_context_from_ros,
)

__all__ = [
    "CoppeliaSimContextExtractor",
    "create_context_from_coppelia",
    "RosContextExtractor",
    "create_context_from_ros",
]
```

3. Atualizar `planning_context/__init__.py`:

```python
from planning_context.extractors import (
    CoppeliaSimContextExtractor,
    create_context_from_coppelia,
    RosContextExtractor,
    create_context_from_ros,
)

__all__ = [
    # ... existing ...
    "RosContextExtractor",
    "create_context_from_ros",
]
```

## 🔍 Exemplo de Uso Completo

```python
from planning_context import (
    PlannerContextBuilder,
    world_to_planner_coords,
    planner_to_world_coords,
)
from algorithms.rrt_star import RRTStar

# ===== Criar contexto =====
context = (PlannerContextBuilder()
    .set_robot_pose(1.0, 1.0, 0.0)
    .set_goal_pose(9.0, 9.0, 0.0)
    .set_world_bounds(0.0, 10.0, 0.0, 10.0)
    .add_obstacle(5.0, 5.0, 2.0, 2.0, "wall_1")
    .set_map_size_pixels(100, 100)
    .build())

# ===== Converter para planejador =====
start = world_to_planner_coords(*context.robot_position(), context)
goal = world_to_planner_coords(*context.goal_position(), context)
obstacles = context.get_obstacles_as_rects()

# ===== Planejar =====
planner = RRTStar(
    start=start,
    goal=goal,
    obstacles=obstacles,
    map_size=context.map_size_pixels,
)
path = planner.planning()

# ===== Converter resultado =====
world_path = [planner_to_world_coords(p[0], p[1], context) for p in path]
```

## 📚 Hierarquia de Dependências

```
planning_context/__init__.py  (interface pública)
        ↓
    ┌───┴─────────────────┬─────────────────┐
    ↓                     ↓                 ↓
core.py         context.py          coordinator.py
(tipos)         (contexto)           (conversões)
                    ↓
              extractors/__init__.py
                    ↓
            extractors/coppelia.py
            (adaptador específico)
```

## ✅ Checklist para Manter Clean Code

- [ ] **SRP**: Arquivo tem uma única responsabilidade?
- [ ] **Interface pública**: Importações vêm de `planning_context/__init__.py`?
- [ ] **Nomeação**: Nomes claros e descritivos?
- [ ] **Documentação**: Docstrings em todas as classes/funções públicas?
- [ ] **Tipos**: Type hints em função pública?
- [ ] **Teste**: Possível usar sem conhecer implementação interna?

## 🚀 Próximos Passos

1. **Adicionar testes unitários**:
   ```
   tests/
   ├── test_core.py
   ├── test_context.py
   ├── test_coordinator.py
   └── test_extractors.py
   ```

2. **Adicionar validações mais rigorosas** em `context.validate()`

3. **Criar mais extractors** para diferentes simuladores/sensores

4. **Documentação completa** com exemplos em Sphinx

---

**Princípio**: Se você precisa conhecer detalhes de implementação interna, o módulo não é limpo o suficiente.
