prim_cam_path = "/Jackal/base_link/Jackal_Camera"    # 左相機的Prim路徑
# ------------------------------------------------------------

# 啟動Isaac Sim
from omni.isaac.kit import SimulationApp

# 選擇是否以headless模式運行
simulation_app = SimulationApp({"headless": False})

import omni
import omni.graph.core as og
import usdrt.Sdf
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, nucleus, stage
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Usd, UsdGeom

# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")

# 加載已有場景
from omni.isaac.core.utils.stage import open_stage
file_path = "/home/user/Documents/fallen/Collected_test/test.usd"
open_stage(usd_path=file_path)

from omni.isaac.core import World
world = World()

import numpy as np
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
camera = Camera(
    prim_path=prim_cam_path,
    position=np.array([-3.11, -1.87, 1.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
)
# 動態建構Action Graph，發布相機數據
import omni.graph.core as og
keys = og.Controller.Keys

# ROS2 cammera action graph (modify from Iassc official example)
(ros_camera_graph, _, _, _) = og.Controller.edit(
    {
        "graph_path": "/publish_camera",
        "evaluator_name": "push",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnTick"),
            ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
            ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
            ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
            ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
            ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
            ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
            ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
            ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
            ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
            ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
            ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
        ],
        keys.SET_VALUES: [
            ("createViewport.inputs:viewportId", 0),
            ("cameraHelperRgb.inputs:frameId", "sim_camera"),
            ("cameraHelperRgb.inputs:topicName", "rgb"),
            ("cameraHelperRgb.inputs:type", "rgb"),
            ("cameraHelperInfo.inputs:frameId", "sim_camera"),
            ("cameraHelperInfo.inputs:topicName", "camera_info"),
            ("cameraHelperInfo.inputs:type", "camera_info"),
            ("cameraHelperDepth.inputs:frameId", "sim_camera"),
            ("cameraHelperDepth.inputs:topicName", "depth"),
            ("cameraHelperDepth.inputs:type", "depth"),
            ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(prim_cam_path)]),
        ],
    },
)

# 運行一次構造的Graph，生成SDGPipeline
# Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
og.Controller.evaluate_sync(ros_camera_graph)

simulation_app.update()

# # Inside the SDGPipeline graph, Isaac Simulation Gate nodes are added to control the execution rate of each of the ROS image and camera info publishers.
# # By default the step input of each Isaac Simulation Gate node is set to a value of 1 to execute every frame.
# # We can change this value to N for each Isaac Simulation Gate node individually to publish every N number of frames.
viewport_api = get_active_viewport()

if viewport_api is not None:
    import omni.syntheticdata._syntheticdata as sd

    # Get name of rendervar for RGB sensor type
    rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)

    # Get path to IsaacSimulationGate node in RGB pipeline
    rgb_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv_rgb + "IsaacSimulationGate", viewport_api.get_render_product_path()
    )

    # Get name of rendervar for DistanceToImagePlane sensor type
    rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )

    # Get path to IsaacSimulationGate node in Depth pipeline
    depth_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv_depth + "IsaacSimulationGate", viewport_api.get_render_product_path()
    )

    # Get path to IsaacSimulationGate node in CameraInfo pipeline
    camera_info_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatch" + "IsaacSimulationGate", viewport_api.get_render_product_path()
    )

    # Set Rgb execution step to 5 frames
    rgb_step_size = 5

    # Set Depth execution step to 60 frames
    depth_step_size = 60

    # Set Camera info execution step to every frame
    info_step_size = 1

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    og.Controller.attribute(rgb_camera_gate_path + ".inputs:step").set(rgb_step_size)
    og.Controller.attribute(depth_camera_gate_path + ".inputs:step").set(depth_step_size)
    og.Controller.attribute(camera_info_gate_path + ".inputs:step").set(info_step_size)

simulation_app.update()
# 根據官方文檔建議，添加完物體之後，最好重置刷新一下世界
print("simulation started")
world.reset()

while simulation_app.is_running() :
    world.step(render=True)

simulation_app.close()
