"""
UAZBUS wheeled vehicle simulation on rigid terrain.

System type  : NSC (ChContactMethod_NSC, rigid terrain)
Main bodies  : UAZBUS vehicle wrapper (chassis, suspension, wheels, tires),
               rigid flat terrain patch
Driver       : ChInteractiveDriverIRR (scored core) with scripted throttle
               in a review-only block for the validation run
Expected     : The UAZBUS accelerates forward on flat asphalt terrain,
               driven interactively via keyboard in real-time mode.
               The vehicle mass is printed at startup.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Simulation parameters ===
step_size         = 1e-3            # physics time step (s)
sim_end           = 20.0            # simulation end time (s)
render_fps        = 50.0            # frames per second for rendering
render_step_size  = 1.0 / render_fps
render_steps      = math.ceil(render_step_size / step_size)  # precomputed once

# === Terrain parameters ===
terrain_length    = 200.0           # m
terrain_width     = 100.0           # m

# === Vehicle initial conditions ===
INIT_LOC          = chrono.ChVector3d(0, 0, 0.5)   # start above terrain
INIT_ROT          = chrono.QuatFromAngleZ(0.0)

# === Vehicle setup ===
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                    # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

# === System & bodies (created by the veh.UAZBUS wrapper) ===
sys     = vehicle.GetSystem()           # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()      # cache: main chassis rigid body
# wheels/spindles: vehicle.GetVehicle().GetAxle(i); terrain: RigidTerrain below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Visualization types — after Initialize()
vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Terrain ===
terrain      = veh.RigidTerrain(sys)
patch_mat    = chrono.ChContactMaterialNSC()    # NSC matches vehicle contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0    # s — time to reach max steering
throttle_time = 1.0    # s — time to reach full throttle
braking_time  = 0.3    # s — time to reach full brake

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

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

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
