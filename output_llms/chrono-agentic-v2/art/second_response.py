"""
ARTcar vehicle simulation on rigid terrain using PyChrono 9.0.x with Irrlicht.

System: NSC (non-smooth contact), wrapper-managed ARTcar vehicle.
Bodies: ARTcar chassis, four wheels/tires, rigid terrain patch.
Configuration:
  - Initial vehicle location: (1, 0, 0.5)
  - Visualization type: PRIMITIVES for all vehicle parts
  - Chassis collision type: MESH
  - Tire model: FIALA
Expected behavior: Vehicle accelerates forward on flat rigid terrain,
  steered interactively via keyboard in real-time mode.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Constants ===
step_size = 1e-3
sim_end = 20.0
render_fps = 50.0
render_steps = math.ceil(1.0 / (render_fps * step_size))  # precomputed once

terrainLength = 200.0
terrainWidth = 200.0

init_loc = chrono.ChVector3d(1, 0, 0.5)
init_rot = chrono.QUNIT

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

# === Data paths (mandatory for catalog vehicle truth) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle ===
vehicle = veh.ARTcar()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_MESH)   # prompt: chassis collision = MESH
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_FIALA)              # prompt: FIALA tire model
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

# === System & bodies (created by the veh.ARTcar wrapper) ===
sys = vehicle.GetSystem()                    # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()           # cache: main chassis rigid body
# wheels/spindles: vehicle.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Visualization types (after Initialize) ===
vehicle.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)    # prompt
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetTireVisualizationType(chrono.VisualizationType_PRIMITIVES)

# === Terrain ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrainLength,
    terrainWidth,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization (Irrlicht vehicle visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar - PRIMITIVES vis / FIALA tire / MESH chassis collision")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)
vis.Initialize()                                  # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                         # vehicle demos use directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_steps * step_size / steering_time)
driver.SetThrottleDelta(render_steps * step_size / throttle_time)
driver.SetBrakingDelta(render_steps * step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)


        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
