from omni.isaac.kit import SimulationApp

# This is a simplified example. A real Isaac Sim script would be more complex.
# This script is intended to be a placeholder for the actual simulation code.

CONFIG = {
    "width": 1280,
    "height": 720,
    "headless": False,
}

simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid

world = World()
world.scene.add_default_ground_plane()

fancy_cube = world.scene.add(
    VisualCuboid(
        prim_path="/new_fancy_cube",
        name="fancy_cube",
        position=[0, 0, 1.0],
        scale=[0.5, 0.5, 0.5],
        color=[0, 0, 1.0],
    )
)

world.reset()

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
