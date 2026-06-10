"""HMMWV on a multi-patch RigidTerrain (NSC, Z-up).

Models a full-size HMMWV (veh.HMMWV_Full, TMEASY tires) driving on a rigid
terrain assembled from four distinct patches:
  - a flat tiled patch,
  - a flat concrete patch (slightly raised),
  - a mesh patch built from the `bump.obj` wavefront mesh, located at
    (0, -42, 0), colored (0.5, 0.5, 0.8) and textured with dirt.jpg
    (UV scaling 6.0, 6.0),
  - a heightmap patch (bump64.bmp) textured with grass.

The vehicle is steered by an interactive Irrlicht driver. Expected behavior:
the HMMWV rests on the rigid patches under gravity and drives forward when
throttle is applied, with all four terrain patches rendered in the scene.
"""

import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
step_size = 2e-3            # integration step (s)
tire_step_size = 1e-3       # tire model sub-step (s)
sim_end = 12.0              # bounded recording horizon (s)
render_fps = 50.0           # review render cadence (frames/s)

# Mesh patch (the bump.obj patch) placement + appearance.
BUMP_PATCH_POS = chrono.ChVector3d(0, -42, 0)      # mesh-patch world location
BUMP_PATCH_COLOR = chrono.ChColor(0.5, 0.5, 0.8)   # bluish tint
BUMP_TEX_SCALE = 6.0                               # dirt.jpg UV tiling

init_loc = chrono.ChVector3d(-10, -2, 0.6)         # HMMWV chassis spawn
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)        # identity orientation

# === Data paths === locate bundled Chrono + vehicle assets
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === HMMWV_Full wrapper owns its ChSystemNSC + chassis/wheels/joints
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC pairs with rigid terrain
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_NONE)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# System + body handles created by the wrapper (made visible for review).
system = hmmwv.GetSystem()                       # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED with contact
chassis = hmmwv.GetChassisBody()                 # cache: main chassis rigid body, reused
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain === rigid terrain assembled from four distinct patches
terrain = veh.RigidTerrain(system)

# Patch 1 — flat tiled ground.
patch1_mat = chrono.ChContactMaterialNSC()
patch1_mat.SetFriction(0.9)
patch1_mat.SetRestitution(0.01)
patch1 = terrain.AddPatch(patch1_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(-16, 0, 0), chrono.QUNIT),
                          32, 20)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

# Patch 2 — flat concrete ground, slightly raised.
patch2_mat = chrono.ChContactMaterialNSC()
patch2_mat.SetFriction(0.9)
patch2_mat.SetRestitution(0.01)
patch2 = terrain.AddPatch(patch2_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(16, 0, 0.15), chrono.QUNIT),
                          32, 30)
patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)

# Patch 3 — mesh patch from bump.obj at (0, -42, 0): bluish tint, dirt texture.
patch3_mat = chrono.ChContactMaterialNSC()
patch3_mat.SetFriction(0.9)
patch3_mat.SetRestitution(0.01)
patch3 = terrain.AddPatch(patch3_mat,
                          chrono.ChCoordsysd(BUMP_PATCH_POS, chrono.QUNIT),
                          veh.GetDataFile("terrain/meshes/bump.obj"))
patch3.SetColor(BUMP_PATCH_COLOR)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), BUMP_TEX_SCALE, BUMP_TEX_SCALE)

# Patch 4 — heightmap patch (bump64.bmp), grass textured.
patch4_mat = chrono.ChContactMaterialNSC()
patch4_mat.SetFriction(0.9)
patch4_mat.SetRestitution(0.01)
patch4 = terrain.AddPatch(patch4_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(0, 42, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/height_maps/bump64.bmp"), 64.0, 64.0, 0.0, 3.0)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6.0, 6.0)

terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window + chase camera + sky + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Rigid Terrain Demo')
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

# === Main loop === real-time render-throttled Synchronize/Advance of the full stack

render_steps = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once: steps per frame
frame = 0
step_number = 0
try:                                              # guard the run so partial output flushes
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        # Draw scene (throttled to the render cadence).
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        step_number += 1

        # Get driver inputs for this step.
        driver_inputs = driver.GetInputs()


        # Update modules in the fixed Synchronize order.
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance all modules by one step.
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)
except (RuntimeError, ValueError) as exc:         # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
