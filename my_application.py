## 一些参数
prim_cam_left_path = "/our_car/body/Camera_left"    # 左相机的Prim路径
topic_name_left_image = "/rgb_left"     # 发布的左影像Topic名称
# ------------------------------------------------------------

# standalone模式规定代码
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# 启用ROS Bridge扩展，否则Isaac Sim无法发布ROS Topic
from omni.isaac.core.utils import extensions
extensions.enable_extension("omni.isaac.ros_bridge")

# 加载环境与机器人(要在新建世界之前，不然会报错)
from omni.isaac.core.utils.stage import open_stage
open_stage(usd_path= "/home/user/.local/share/ov/pkg/isaac-sim-2023.1.1/exts/omni.isaac.examples/omni/isaac/examples/IsaacSim_ObjectDetection_learning/A-Car-with-Stereo-and-IMU-for-Isaac-Sim/our_carV6.usd")

# 新建世界
from omni.isaac.core import World
world = World()

# 构造Action Graph，发布相机数据
import omni.graph.core as og
keys = og.Controller.Keys
(ros_camera_graph, _, _, _) = og.Controller.edit(
    {
        "graph_path": "/publish_camera",    # 注意Graph的名称必须以/开头
        "evaluator_name": "push",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("createViewportLeft", "omni.isaac.core_nodes.IsaacCreateViewport"),
            ("setActiveCameraLeft", "omni.graph.ui.SetActiveViewportCamera"),
            ("cameraHelperLeft", "omni.isaac.ros_bridge.ROS1CameraHelper"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "createViewportLeft.inputs:execIn"),
            ("createViewportLeft.outputs:viewport", "setActiveCameraLeft.inputs:execIn"),
            ("createViewportLeft.outputs:viewport", "cameraHelperLeft.inputs:viewport"),
            ("setActiveCameraLeft.outputs:execOut", "cameraHelperLeft.inputs:execIn"),
        ],
        keys.SET_VALUES: [
            ("createViewportLeft.inputs:viewportId", 1),
            ("setActiveCameraLeft.inputs:primPath", prim_cam_left_path),

            ("cameraHelperLeft.inputs:topicName", topic_name_left_image),
            ("cameraHelperLeft.inputs:type", "rgb"),
        ],
    },
)

# 运行一次构造的Graph，生成SDGPipeline
og.Controller.evaluate_sync(ros_camera_graph)
simulation_app.update()

print("start to simulate")
world.reset()

# 开始循环
while True:
    world.step(render=True)