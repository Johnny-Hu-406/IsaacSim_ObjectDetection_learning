## 一些参数
prim_cam_left_path = "/our_car/body/Camera_left"    # 左相机的Prim路径

# ------------------------------------------------------------

# 启动Isaac Sim，对于Standalone方式而言是必须的
from omni.isaac.kit import SimulationApp

# 可以选择是否以headless模式运行
simulation_app = SimulationApp({"headless": False})

# 启用ROS Bridge扩展，否则Isaac Sim无法发布ROS Topic
import omni
import omni.graph.core as og
import usdrt.Sdf
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, nucleus, stage
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Usd, UsdGeom

# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")

# 加载已有场景
from omni.isaac.core.utils.stage import open_stage
file_path = "/home/user/Documents/fallen/Collected_test/test.usd"
open_stage(usd_path=file_path)

# 添加世界
from omni.isaac.core import World
world = World()

# 根据官方文档建议，添加完物体之后，最好重置刷新一下世界
print("simulation started")
world.reset()

while simulation_app.is_running() :
    world.step(render=True)

simulation_app.close()
