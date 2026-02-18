from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class Node:
    x: float
    y: float
    parent: Optional["Node"] = None
    cost: float = 0.0

    @property
    def pos(self) -> Tuple[float, float]:
        return (self.x, self.y)


def build_path(node: Optional[Node]) -> List[Tuple[float, float]]:
    path = []
    while node is not None:
        path.append(node.pos)
        node = node.parent
    return path[::-1]


def reconstruct_bidirectional_path(
    node_a: Node, node_b: Node
) -> List[Tuple[float, float]]:
    path_a = build_path(node_a)
    path_b = list(reversed(build_path(node_b)))
    if path_a and path_b and path_a[-1] == path_b[0]:
        path_b = path_b[1:]
    return path_a + path_b
