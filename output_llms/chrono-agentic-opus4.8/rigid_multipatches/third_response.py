"""HMMWV on a multi-patch rigid terrain (PyChrono, NSC system).

Models a full HMMWV wheeled vehicle (TMEASY tires, AWD, simple powertrain)
driving on a RigidTerrain composed of four distinct patches:
  * Patch 1 — flat box patch (tile texture)        centered at (-20, 5, 0)
  * Patch 2 — raised flat box patch (concrete)      centered at (20, -5, 0.2)
  * Patch 3 — triangle-mesh patch (bump.obj, dirt)  centered at (5, -45, 0)
  * Patch 4 — heightmap patch (bump64.bmp, grass)   centered at (10, 40, 0)
The vehicle is controlled by an interactive Irrlicht driver. Expected behavior:
the chassis rests on the terrain at spawn and is steerable in real time; the
four patches sit at their prescribed world positions with their own materials.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants === geometry, timing, and the four patch world positions
step_size = 2e-3
tire_step_size = 1e-3
sim_end = 12.0
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_steps = math.ceil(render_step_size / step_size)                # precomputed once

init_loc = chrono.ChVector3d(-10, -2, 0.6)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

patch1_pos = chrono.ChVector3d(-20, 5, 0)      # flat box patch
patch2_pos = chrono.ChVector3d(20, -5, 0.2)    # raised flat box patch
patch3_pos = chrono.ChVector3d(5, -45, 0)      # triangle-mesh patch
patch4_pos = chrono.ChVector3d(10, 40, 0)      # heightmap patch

# === Data paths === locate bundled Chrono + vehicle assets
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV wrapper (owns its NSC ChSystem)
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                             # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for contact
chassis = hmmwv.GetChassisBody()                       # cache: main chassis body, reused below
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain === RigidTerrain with four distinct patches (box, raised box, mesh, heightmap)
terrain = veh.RigidTerrain(system)

patch1_mat = chrono.ChContactMaterialNSC()
patch1_mat.SetFriction(0.9)
patch1_mat.SetRestitution(0.01)
patch1 = terrain.AddPatch(patch1_mat, chrono.ChCoordsysd(patch1_pos, chrono.QUNIT), 32, 20)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

patch2_mat = chrono.ChContactMaterialNSC()
patch2_mat.SetFriction(0.9)
patch2_mat.SetRestitution(0.01)
patch2 = terrain.AddPatch(patch2_mat, chrono.ChCoordsysd(patch2_pos, chrono.QUNIT), 32, 30)
patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)

patch3_mat = chrono.ChContactMaterialNSC()
patch3_mat.SetFriction(0.9)
patch3_mat.SetRestitution(0.01)
patch3 = terrain.AddPatch(patch3_mat, chrono.ChCoordsysd(patch3_pos, chrono.QUNIT),
                          veh.GetDataFile("terrain/meshes/bump.obj"))
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

patch4_mat = chrono.ChContactMaterialNSC()
patch4_mat.SetFriction(0.9)
patch4_mat.SetRestitution(0.01)
patch4 = terrain.AddPatch(patch4_mat, chrono.ChCoordsysd(patch4_pos, chrono.QUNIT),
                          veh.GetDataFile("terrain/height_maps/bump64.bmp"), 64.0, 64.0, 0.0, 3.0)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6.0, 6.0)

terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window: sky + directional light + chase camera
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Rigid Multi-Patch Terrain Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive Irrlicht driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

hmmwv.GetVehicle().EnableRealtime(True)

# === Main loop === real-time render-throttled Synchronize/Advance over the subsystem stack

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:             # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
