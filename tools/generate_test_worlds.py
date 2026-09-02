#!/usr/bin/env python3

"""Generate matched Gazebo worlds and Nav2 occupancy maps."""

from dataclasses import dataclass
from math import ceil, floor
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORLD_DIR = ROOT / 'worlds'
MAP_DIR = ROOT / 'map'
RESOLUTION = 0.05
WORLD_WIDTH = 12.0
WORLD_HEIGHT = 8.0
MAP_WIDTH = round(WORLD_WIDTH / RESOLUTION)
MAP_HEIGHT = round(WORLD_HEIGHT / RESOLUTION)
ORIGIN_X = -WORLD_WIDTH / 2.0
ORIGIN_Y = -WORLD_HEIGHT / 2.0


@dataclass(frozen=True)
class Box:
    """Axis-aligned collision box represented in world coordinates."""

    name: str
    x: float
    y: float
    size_x: float
    size_y: float
    height: float = 0.8


@dataclass(frozen=True)
class Cylinder:
    """Vertical collision cylinder represented in world coordinates."""

    name: str
    x: float
    y: float
    radius: float
    height: float = 0.8


BOUNDARY_WALLS = (
    Box('north_wall', 0.0, 3.9, 12.0, 0.2),
    Box('south_wall', 0.0, -3.9, 12.0, 0.2),
    Box('east_wall', 5.9, 0.0, 0.2, 8.0),
    Box('west_wall', -5.9, 0.0, 0.2, 8.0),
)

SCENARIOS = {
    'open_arena': {
        'color': (0.32, 0.50, 0.72),
        'boxes': (),
        'cylinders': (),
    },
    'corridor': {
        'color': (0.62, 0.46, 0.27),
        'boxes': (
            Box('north_stagger', -2.0, 1.0, 5.0, 0.2),
            Box('south_stagger', 2.0, -1.0, 5.0, 0.2),
            Box('west_gate', -3.0, -1.8, 0.2, 2.0),
            Box('east_gate', 3.0, 1.8, 0.2, 2.0),
        ),
        'cylinders': (),
    },
    'obstacle_course': {
        'color': (0.50, 0.34, 0.62),
        'boxes': (
            Box('northwest_block', -3.0, 1.8, 1.2, 0.8),
            Box('southwest_block', -2.0, -2.0, 0.8, 1.2),
            Box('north_center_block', 0.0, 1.4, 0.8, 1.8),
            Box('northeast_block', 2.7, 1.6, 1.4, 0.6),
            Box('southeast_block', 3.0, -1.7, 0.8, 1.4),
        ),
        'cylinders': (
            Cylinder('south_center_pillar', 0.3, -1.6, 0.45),
            Cylinder('west_pillar', -4.0, -0.2, 0.5),
        ),
    },
}


def _number(value: float) -> str:
    return f'{value:.3f}'.rstrip('0').rstrip('.')


def _box_sdf(box: Box, color: tuple[float, float, float]) -> str:
    pose = f'{_number(box.x)} {_number(box.y)} {_number(box.height / 2)} 0 0 0'
    size = f'{_number(box.size_x)} {_number(box.size_y)} {_number(box.height)}'
    diffuse = ' '.join(_number(component) for component in (*color, 1.0))
    return f"""      <collision name="{box.name}_collision">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
      </collision>
      <visual name="{box.name}_visual">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <ambient>{diffuse}</ambient>
          <diffuse>{diffuse}</diffuse>
        </material>
      </visual>"""


def _cylinder_sdf(
    cylinder: Cylinder, color: tuple[float, float, float]
) -> str:
    pose = (
        f'{_number(cylinder.x)} {_number(cylinder.y)} '
        f'{_number(cylinder.height / 2)} 0 0 0'
    )
    diffuse = ' '.join(_number(component) for component in (*color, 1.0))
    return f"""      <collision name="{cylinder.name}_collision">
        <pose>{pose}</pose>
        <geometry>
          <cylinder>
            <radius>{_number(cylinder.radius)}</radius>
            <length>{_number(cylinder.height)}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="{cylinder.name}_visual">
        <pose>{pose}</pose>
        <geometry>
          <cylinder>
            <radius>{_number(cylinder.radius)}</radius>
            <length>{_number(cylinder.height)}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{diffuse}</ambient>
          <diffuse>{diffuse}</diffuse>
        </material>
      </visual>"""


def _world_sdf(name: str, scenario: dict) -> str:
    obstacles = (*BOUNDARY_WALLS, *scenario['boxes'])
    geometry = [_box_sdf(box, scenario['color']) for box in obstacles]
    geometry.extend(
        _cylinder_sdf(cylinder, scenario['color'])
        for cylinder in scenario['cylinders']
    )
    geometry_xml = '\n'.join(geometry)
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="{name}">
    <physics name="accurate_1ms" type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <sor>1.4</sor>
          <use_dynamic_moi_rescaling>true</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.0</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>

    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.7 0.7 0.7 1</ambient>
      <background>0.85 0.9 0.95 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{_number(WORLD_WIDTH)} {_number(WORLD_HEIGHT)}</size>
            </plane>
          </geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{_number(WORLD_WIDTH)} {_number(WORLD_HEIGHT)}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.72 0.72 0.72 1</ambient>
            <diffuse>0.82 0.82 0.82 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="arena_geometry">
      <static>true</static>
      <link name="geometry_link">
{geometry_xml}
      </link>
    </model>
  </world>
</sdf>
"""


def _paint_box(pixels: bytearray, box: Box) -> None:
    x_min = floor((box.x - box.size_x / 2.0 - ORIGIN_X) / RESOLUTION)
    x_max = ceil((box.x + box.size_x / 2.0 - ORIGIN_X) / RESOLUTION)
    y_min = floor((box.y - box.size_y / 2.0 - ORIGIN_Y) / RESOLUTION)
    y_max = ceil((box.y + box.size_y / 2.0 - ORIGIN_Y) / RESOLUTION)
    for map_y in range(max(0, y_min), min(MAP_HEIGHT, y_max)):
        row = MAP_HEIGHT - 1 - map_y
        for column in range(max(0, x_min), min(MAP_WIDTH, x_max)):
            pixels[row * MAP_WIDTH + column] = 0


def _paint_cylinder(pixels: bytearray, cylinder: Cylinder) -> None:
    radius_cells = ceil(cylinder.radius / RESOLUTION)
    center_x = (cylinder.x - ORIGIN_X) / RESOLUTION
    center_y = (cylinder.y - ORIGIN_Y) / RESOLUTION
    radius_squared = cylinder.radius * cylinder.radius
    for map_y in range(
        max(0, floor(center_y - radius_cells)),
        min(MAP_HEIGHT, ceil(center_y + radius_cells)),
    ):
        world_y = ORIGIN_Y + (map_y + 0.5) * RESOLUTION
        row = MAP_HEIGHT - 1 - map_y
        for column in range(
            max(0, floor(center_x - radius_cells)),
            min(MAP_WIDTH, ceil(center_x + radius_cells)),
        ):
            world_x = ORIGIN_X + (column + 0.5) * RESOLUTION
            distance_squared = (
                (world_x - cylinder.x) ** 2
                + (world_y - cylinder.y) ** 2
            )
            if distance_squared <= radius_squared:
                pixels[row * MAP_WIDTH + column] = 0


def _write_map(name: str, scenario: dict) -> None:
    pixels = bytearray([254]) * (MAP_WIDTH * MAP_HEIGHT)
    for box in (*BOUNDARY_WALLS, *scenario['boxes']):
        _paint_box(pixels, box)
    for cylinder in scenario['cylinders']:
        _paint_cylinder(pixels, cylinder)

    pgm_path = MAP_DIR / f'{name}.pgm'
    header = f'P5\n{MAP_WIDTH} {MAP_HEIGHT}\n255\n'.encode('ascii')
    pgm_path.write_bytes(header + pixels)

    yaml_path = MAP_DIR / f'{name}.yaml'
    yaml_path.write_text(
        f"""image: {name}.pgm
resolution: {RESOLUTION:.6f}
origin: [{ORIGIN_X:.6f}, {ORIGIN_Y:.6f}, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""",
        encoding='utf-8',
    )


def main() -> None:
    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    MAP_DIR.mkdir(parents=True, exist_ok=True)
    for name, scenario in SCENARIOS.items():
        (WORLD_DIR / f'{name}.world').write_text(
            _world_sdf(name, scenario), encoding='utf-8'
        )
        _write_map(name, scenario)
        print(f'generated {name}.world and {name}.{{yaml,pgm}}')


if __name__ == '__main__':
    main()
