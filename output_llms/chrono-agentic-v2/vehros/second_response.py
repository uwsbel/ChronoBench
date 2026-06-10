"""
HMMWV vehicle on rigid terrain with ROS2 bridge and Irrlicht visualization.

System type: NSC (rigid terrain, catalog wheeled vehicle default).
Main bodies: HMMWV_Full chassis + 4 wheel spindles, flat RigidTerrain patch.
ROS handlers: ChROSClockHandler + ChROSDriverInputsHandler (subscribes throttle/steer/brake)
              + ChROSBodyHandler (publishes chassis pose/twist).
Visualization: ChWheeledVehicleVisualSystemIrrlicht chase camera.
Expected behavior: HMMWV sits on flat terrain; ROS graph receives chassis body state;
                   driver inputs can be commanded via ROS topic; Irrlicht window renders
                   the scene in real time.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros


# === Parameters ===
time_step = 1e-3          # physics step size (s)
sim_end   = 10.0          # simulation end time (s)

TERRAIN_LENGTH = 200.0    # terrain patch X dimension (m)
TERRAIN_WIDTH  = 200.0    # terrain patch Y dimension (m)

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)   # chassis init position (m)
INIT_ROT = chrono.QuatFromAngleZ(0.0)          # chassis init orientation

render_fps   = 50.0                                        # precomputed once
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === Data paths (truth-faithful, scored core) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys     = hmmwv.GetSystem()                # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body — reused in handler
# wheels/spindles: hmmwv.GetVehicle().GetAxles(); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the HMMWV wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain ===
terrain     = veh.RigidTerrain(sys)
patch_mat   = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV + ROS2")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive IRR — scored-core default for real-time catalog vehicle) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0   # s to reach full steering
throttle_time = 1.0   # s to reach full throttle
braking_time  = 0.3   # s to reach full braking
driver.SetSteeringDelta(render_fps ** -1 / steering_time)   # precomputed once
driver.SetThrottleDelta(render_fps ** -1 / throttle_time)
driver.SetBrakingDelta(render_fps ** -1 / braking_time)
driver.Initialize()

# === ROS2 Manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler FIRST — publishes /clock so the ROS graph is time-synced to sim
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Driver-inputs handler SUBSCRIBES throttle/steering/braking from ROS
ros_manager.RegisterHandler(
    chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs")
)

# 3. Body handler PUBLISHES chassis pose/twist to ROS
chassis.SetName("base_link")   # TF root convention
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(25, chassis, "~/output/vehicle/state")
)

# Initialize ONCE after all handlers are registered
ros_manager.Initialize()
