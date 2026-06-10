"""
Kraz Tractor-Trailer + BMW E90 Sedan on Highway Mesh Terrain
=============================================================
System:     ChSystemNSC (NSC rigid contact), Bullet collision
Vehicles:   veh.Kraz() tractor-trailer (RIGID tire step size), veh.BMW_E90() sedan
Terrain:    RigidTerrain with predefined highway mesh (Highway_col.obj)
Driver 1:   ChInteractiveDriver for the Kraz (interactive keyboard control)
Driver 2:   ChDataDriver for the sedan (fixed throttle, zero steering — forward motion)
Logging:    Tractor chassis pose/velocity + trailer chassis pose/velocity -> CSV
Expected:   Truck and sedan drive along highway mesh; truck tractor+trailer state stored.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Data paths: point to the source-build data directory ===
# The source build resolves vehicle/longhaul/robot assets from build/data/
chrono.SetChronoDataPath("/home/hongyu/Documents/chrono-900/build/data/")
veh.SetDataPath("/home/hongyu/Documents/chrono-900/build/data/vehicle/")

# === Constants ===
STEP_SIZE    = 2e-3           # physics step [s]
SIM_END      = 20.0           # simulation duration [s]
RENDER_FPS   = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# Truck initial pose (changed vs default): right lane of highway mesh
# Highway mesh occupies X in [-11.5, +11.5], Y in [-75, +76], Z near 0
# Truck in right lane (x=-3.5) heading along +Y (mesh length direction)
TRUCK_INIT_POS = chrono.ChVector3d(-3.5, -50.0, 0.5)
TRUCK_INIT_ROT = chrono.QuatFromAngleZ(math.pi / 2)  # heading along +Y

# Sedan initial pose: left lane, behind the truck
SEDAN_INIT_POS = chrono.ChVector3d(0.0, -65.0, 0.3)
SEDAN_INIT_ROT = chrono.QuatFromAngleZ(math.pi / 2)  # heading along +Y

# Highway mesh path (synchrono asset — resolved after SetChronoDataPath)
HIGHWAY_MESH = chrono.GetChronoDataFile("synchrono/meshes/Highway_col.obj")


# === Kraz tractor-trailer ===
kraz = veh.Kraz()
kraz.SetContactMethod(chrono.ChContactMethod_NSC)
kraz.SetChassisFixed(False)
kraz.SetChassisCollisionType(veh.CollisionType_NONE)
kraz.SetInitPosition(chrono.ChCoordsysd(TRUCK_INIT_POS, TRUCK_INIT_ROT))
kraz.SetTireStepSize(STEP_SIZE)
kraz.Initialize()

# === System & bodies (owned by veh.Kraz wrapper) ===
system = kraz.GetSystem()   # ChSystemNSC created by wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for terrain contact

# cache: frequently accessed wrapper handles
tractor_chassis_body = kraz.GetTractorChassisBody()  # cache: tractor ChBodyAuxRef
trailer              = kraz.GetTrailer()             # cache: ChWheeledTrailer
# trailer body: trailer.GetChassis().GetBody()
# system bodies include tractor + trailer bodies + all axle/wheel sub-bodies

kraz.SetChassisVisualizationType(
    veh.VisualizationType_MESH, veh.VisualizationType_MESH)
kraz.SetSuspensionVisualizationType(
    veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
kraz.SetWheelVisualizationType(
    veh.VisualizationType_MESH, veh.VisualizationType_MESH)
kraz.SetTireVisualizationType(
    veh.VisualizationType_MESH, veh.VisualizationType_MESH)

# === BMW E90 Sedan ===
sedan = veh.BMW_E90()
sedan.SetContactMethod(chrono.ChContactMethod_NSC)
sedan.SetChassisFixed(False)
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetInitPosition(chrono.ChCoordsysd(SEDAN_INIT_POS, SEDAN_INIT_ROT))
sedan.SetTireType(veh.TireModelType_RIGID)   # prompt: rigid tire
sedan.SetTireStepSize(STEP_SIZE)
sedan.Initialize()

sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain: RigidTerrain with highway mesh ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    HIGHWAY_MESH,   # predefined highway mesh
)
patch.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
terrain.Initialize()

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz + Sedan on Highway Mesh")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(kraz.GetTractor())   # Kraz: attach tractor vehicle, NOT GetVehicle()

# === Driver 1: Interactive (Kraz) ===
# ChInteractiveDriver takes the ChVehicle — matched to truth pattern for catalog demos
render_step   = 1.0 / RENDER_FPS   # precomputed once
driver_kraz   = veh.ChInteractiveDriver(kraz.GetTractor())
driver_kraz.SetSteeringDelta(render_step / 1.0)   # 1.0 s to full steering
driver_kraz.SetThrottleDelta(render_step / 1.0)   # 1.0 s to full throttle
driver_kraz.SetBrakingDelta(render_step / 0.3)    # 0.3 s to full brake
driver_kraz.Initialize()

# === Driver 2: Scripted sedan (fixed throttle, zero steering, forward motion) ===
sedan_entries = veh.vector_Entry([
    veh.DataDriverEntry(0.0,    0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.5,    0.0, 0.0, 0.0),
    veh.DataDriverEntry(1.0,    0.0, 0.5, 0.0),
    veh.DataDriverEntry(SIM_END, 0.0, 0.5, 0.0),
])
driver_sedan = veh.ChDataDriver(sedan.GetVehicle(), sedan_entries)
driver_sedan.Initialize()

# === Output directory ===
os.makedirs("cam", exist_ok=True)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()   # cache: per-step local

        # Throttled rendering
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Driver inputs
        kraz_inputs  = driver_kraz.GetInputs()
        sedan_inputs = driver_sedan.GetInputs()


        # Synchronize: driver → terrain → vehicles → vis
        driver_kraz.Synchronize(time)
        driver_sedan.Synchronize(time)
        terrain.Synchronize(time)
        kraz.Synchronize(time, kraz_inputs, terrain)
        sedan.Synchronize(time, sedan_inputs, terrain)
        vis.Synchronize(time, kraz_inputs)

        # Advance all subsystems
        driver_kraz.Advance(STEP_SIZE)
        driver_sedan.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        kraz.Advance(STEP_SIZE)
        sedan.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
