"""UAZBUS wheeled-vehicle mobility test on flat rigid terrain (PyChrono 9.0.0, Irrlicht).

Models the catalog UAZBUS (veh.UAZBUS) driving forward on a flat RigidTerrain
patch with RIGID tires, under a constant throttle of 0.5. A fixed box obstacle
(0.5 x 5 x 0.2 m at (5, 0, 0.1)) is placed ahead of the vehicle to probe its
mobility. System type is NSC (rigid-terrain catalog-vehicle default); contact is
resolved by the Bullet collision system. Expected behavior: the bus accelerates
forward and interacts with the box obstacle in its path.
"""

import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0

init_x = 0.0
init_y = 0.0
SUSPENSION_REF_HEIGHT = 0.45        # UAZBUS chassis origin above wheel-bottom at rest
TERRAIN_TOP_Z = 0.0
init_z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT

terrain_length = 100.0
terrain_width = 100.0

# Box obstacle (full extents) placed in the vehicle's forward path.
box_size = chrono.ChVector3d(0.5, 5.0, 0.2)
box_pos = chrono.ChVector3d(5.0, 0.0, 0.1)

throttle_value = 0.5                 # constant forward throttle

# === Data paths === truth-faithful catalog-vehicle data anchoring
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

# === Vehicle === UAZBUS catalog wrapper (owns its ChSystem) with RIGID tires
init_loc = chrono.ChVector3d(init_x, init_y, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_RIGID)           # prompt: rigid tire model
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.UAZBUS wrapper) ===
system = vehicle.GetSystem()                       # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = vehicle.GetChassisBody()                 # cache: main chassis rigid body, reused below
# wheels/spindles: vehicle.GetVehicle().GetSpindlePos(axle, side); joints: suspension + steering inside wrapper
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Terrain === flat rigid patch the bus drives on
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Box obstacle === fixed box in the forward path to test mobility
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.9)
box_mat.SetRestitution(0.01)
box = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, 1000.0, True, True, box_mat)
box.SetPos(box_pos)
box.SetFixed(True)                                 # prompt: fixed in place
box.SetName("box_obstacle")
box.EnableCollision(True)
system.AddBody(box)
system.GetCollisionSystem().BindAll()   # rebuild models so the box is seen by contact

# Footprint sanity: wheel bottoms must rest on (not through) the terrain.
TIRE_RADIUS = 0.4
veh_obj = vehicle.GetVehicle()
spindle_world = [veh_obj.GetSpindlePos(axle, side)
                 for axle in range(veh_obj.GetNumberAxles())
                 for side in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - 0.1, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT")

# === Driver === scripted constant-throttle forward drive (matches truth shape)
driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()

# === Visualization === full vehicle Irrlicht scene: window + sky + camera + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("UAZBUS mobility test")
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

# === Main loop === throttled rendering; full subsystem Synchronize/Advance stack
render_step_size = 1.0 / render_fps
render_steps = math.ceil(render_step_size / time_step)   # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    step_number = 0
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver.SetThrottle(throttle_value)           # constant forward throttle
        driver.SetSteering(0.0)
        driver.SetBraking(0.0)

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
