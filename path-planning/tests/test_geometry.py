from utils.geometry import is_collision_free, point_in_obstacles, sample_bridge_point


def test_segment_intersects_aabb_obstacle():
    obstacles = [(2, 2, 4, 4)]
    assert not is_collision_free((0, 3), (5, 3), obstacles)


def test_segment_away_from_obstacle_is_free():
    obstacles = [(2, 2, 4, 4)]
    assert is_collision_free((0, 0), (1, 1), obstacles)


def test_point_validation_respects_obstacle_boundary():
    obstacles = [(2, 2, 4, 4)]
    assert point_in_obstacles((3, 3), obstacles)
    assert point_in_obstacles((2, 3), obstacles)
    assert not point_in_obstacles((1, 1), obstacles)


def test_bridge_sampling_returns_none_without_obstacles():
    assert sample_bridge_point((100, 100), [], max_tries=5) is None
