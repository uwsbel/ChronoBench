"""
HMMWV on rigid multipatch terrain — turn 3.

This simulation models an HMMWV (High Mobility Multipurpose Wheeled Vehicle) driving
on a rigid terrain composed of four patches at modified positions (from input3.txt):
  Patch 1: flat tile at (-20, 5, 0)
  Patch 2: flat concrete at (20, -5, 0.2)
  Patch 3: mesh bump at (5, -45, 0)
  Patch 4: heightmap at (10, 40, 0)

System type: ChSystemNSC (Non-Smooth Contact) for rigid body contacts.
The vehicle uses TMEASY tires suitable for rigid terrain.
An interactive driver (ChInteractiveDriverIRR) provides real-time steering/throttle/braking.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === review-only: recording infrastructure ===

# Simulation parameters
step_size = 2e-3
tire_step_size = 1e-3

# Chrono vehicle data path
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Create the HMMWV vehicle ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.QUNIT))
hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

# Visualization types for vehicle components
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# Collision system — REQUIRED for contact/terrain scenes
hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Create the rigid terrain with multiple patches ===
terrain = veh.RigidTerrain(hmmwv.GetSystem())

# --- Patch 1: flat tile terrain at (-20, 5, 0) ---
patch1_mat = chrono.ChContactMaterialNSC()
patch1_mat.SetFriction(0.9)
patch1_mat.SetRestitution(0.01)
patch1 = terrain.AddPatch(
    patch1_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-20, 5, 0), chrono.QUNIT),
    32,
    20,
)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

# --- Patch 2: flat concrete at (20, -5, 0.2) ---
patch2_mat = chrono.ChContactMaterialNSC()
patch2_mat.SetFriction(0.9)
patch2_mat.SetRestitution(0.01)
patch2 = terrain.AddPatch(
    patch2_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(20, -5, 0.2), chrono.QUNIT),
    32,
    30,
)
patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)

# --- Patch 3: mesh bump terrain at (5, -45, 0) ---
patch3_mat = chrono.ChContactMaterialNSC()
patch3_mat.SetFriction(0.9)
patch3_mat.SetRestitution(0.01)
patch3 = terrain.AddPatch(
    patch3_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(5, -45, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"),
)
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# --- Patch 4: heightmap bump terrain at (10, 40, 0) ---
patch4_mat = chrono.ChContactMaterialNSC()
patch4_mat.SetFriction(0.9)
patch4_mat.SetRestitution(0.01)
patch4 = terrain.AddPatch(
    patch4_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(10, 40, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    64.0,
    64.0,
    0.0,
    3.0,
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6.0, 6.0)

terrain.Initialize()

# === Visualization — Irrlicht window + chase camera ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Rigid Terrain Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

hmmwv.GetVehicle().EnableRealtime(True)

# === Review-only: recording setup ===

render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))

# === Main simulation loop ===
frame = 0
while vis.Run() and hmmwv.GetSystem().GetChTime() < 60.0:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        sim_time = hmmwv.GetSystem().GetChTime()
        driver_inputs = driver.GetInputs()


        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        if hmmwv.GetSystem().GetChTime() >= 60.0:
            break
