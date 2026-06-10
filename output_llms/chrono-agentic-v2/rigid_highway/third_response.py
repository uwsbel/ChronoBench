"""
HMMWV on a rigid highway terrain — NSC contact, mesh terrain patch.

System type : ChSystemNSC (owned by HMMWV_Full wrapper)
Main bodies : HMMWV chassis, four wheel/spindle assemblies, rigid terrain patch
Terrain     : RigidTerrain with a highway collision mesh (Highway_col.obj) and
              a separate visual mesh (Highway_vis.obj), patch placed at (6, -70, 0)
              with a -90-degree rotation about the Z axis so the vehicle is at a
              crossroads with the terrain.
Contact mat : NSC, friction=0.4, restitution=0.05
Driver      : ChInteractiveDriverIRR (real-time keyboard / interactive)
Expected    : HMMWV spawns at the highway crossroads and can be driven interactively.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Named constants ===
# Vehicle spawn — centred at the highway crossroads
INIT_LOC = chrono.ChVector3d(6, -70, 0.5)
INIT_ROT = chrono.QuatFromAngleZ(1.57)   # face along highway

# Terrain patch position and orientation (turn-3 change)
PATCH_POS   = chrono.ChVector3d(6, -70, 0)
PATCH_ROT_ANGLE = -math.pi / 2           # -90 deg about Z

# Physics / timing
STEP_SIZE        = 1e-3
TIRE_STEP_SIZE   = STEP_SIZE
SIM_END          = 20.0
RENDER_FPS       = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS      # seconds between render frames

# precomputed once
render_steps = math.ceil(RENDER_STEP_SIZE / STEP_SIZE)  # physics steps per frame

# Camera track point on chassis
TRACK_POINT = chrono.ChVector3d(-3.0, 0.0, 1.1)

# === Vehicle setup (HMMWV_Full wrapper owns ChSystemNSC) ===
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = vehicle.GetSystem()              # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()        # cache: main chassis rigid body; wheels/spindles via vehicle.GetAxle(i)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Terrain — RigidTerrain with highway mesh patch ===
# Contact material: friction=0.4, restitution=0.05 (turn-3 values)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.4)
patch_mat.SetRestitution(0.05)

terrain = veh.RigidTerrain(system)

# Patch orientation: -90 deg about Z axis (turn-3 change)
quat = chrono.ChQuaterniond()
quat.SetFromAngleAxis(PATCH_ROT_ANGLE, chrono.ChVector3d(0, 0, 1))

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(PATCH_POS, quat),
    chrono.GetChronoDataFile('vehicle/terrain/meshes/Highway_col.obj'),
    True, 0.01, False
)

# Visual highway mesh (separate, higher-detail visual mesh)
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    veh.GetDataFile('terrain/meshes/Highway_vis.obj'), True, True
)
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape.SetMesh(vis_mesh)
tri_mesh_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)

terrain.Initialize()

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(TRACK_POINT, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver — interactive IRR (scored-core default, truth-faithful) ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(RENDER_STEP_SIZE / steering_time)
driver.SetThrottleDelta(RENDER_STEP_SIZE / throttle_time)
driver.SetBrakingDelta(RENDER_STEP_SIZE / braking_time)
driver.Initialize()

# === Recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass                                     # review-only writers closed in REC block below
