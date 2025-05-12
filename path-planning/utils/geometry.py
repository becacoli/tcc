import math

def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def steer(from_node, to_point, step_size):
    angle = math.atan2(to_point[1] - from_node[1], to_point[0] - from_node[0])
    new_x = from_node[0] + step_size * math.cos(angle)
    new_y = from_node[1] + step_size * math.sin(angle)
    return (int(new_x), int(new_y))

def is_collision_free(p1, p2, obstacles=[]):
    for obs in obstacles:
        if line_intersects_rect(p1, p2, obs):
            return False
    return True

def line_intersects_rect(p1, p2, rect):
    from shapely.geometry import LineString, box
    line = LineString([p1, p2])
    rect_shape = box(*rect) 
    return line.intersects(rect_shape)
