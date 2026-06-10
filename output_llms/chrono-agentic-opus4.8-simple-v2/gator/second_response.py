import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate bundled vehicle data files

init_loc = chrono.ChVector3d(-15, 0, 0.5)                            # spawn behind the patch field, facing +X
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation
step_size = 2e-3                                                     # integration step
tire_step_size = 1e-3                                                # tire substep
sim_end = 16.0                                                       # total sim time

gator = veh.Gator()                                                 # Gator catalog wrapper (owns its system)
gator.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision shape
gator.SetChassisFixed(False)                                       # MANDATORY — fixed chassis won't move
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # initial chassis pose
gator.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tires (handle bumps/grades)
gator.SetTireStepSize(tire_step_size)                              # tire force substep
gator.Initialize()                                                 # build the vehicle

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)      # mesh chassis
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

system = gator.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                 # rigid multi-patch terrain

# Patch 1 — flat grass patch (vehicle starts here, drives forward in +X)
patch1_mat = chrono.ChContactMaterialNSC()                         # NSC contact material
patch1_mat.SetFriction(0.9)                                        # high grip
patch1_mat.SetRestitution(0.01)                                    # nearly inelastic
patch1 = terrain.AddPatch(patch1_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(-20, 0, 0), chrono.QUNIT),
                          30, 30)                                   # flat 30x30 patch
patch1.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 30, 30)  # grass texture

# Patch 2 — flat dirt patch (different texture) ahead in +X
patch2_mat = chrono.ChContactMaterialNSC()                         # second material
patch2_mat.SetFriction(0.8)                                        # slightly lower grip dirt
patch2_mat.SetRestitution(0.01)
patch2 = terrain.AddPatch(patch2_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(10, 0, 0), chrono.QUNIT),
                          30, 30)                                   # flat 30x30 patch
patch2.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 30, 30)   # dirt texture

# Patch 3 — heightmap slope patch for GRADABILITY testing (vehicle climbs the grade)
patch3_mat = chrono.ChContactMaterialNSC()                         # third material
patch3_mat.SetFriction(0.9)                                        # grip needed to climb
patch3_mat.SetRestitution(0.01)
patch3 = terrain.AddPatch(patch3_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(35, 0, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/height_maps/slope.bmp"),
                          30, 30, 0.0, 4.0)                         # heightmap: length, width, hMin, hMax -> a 4 m grade
patch3.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 30, 30)  # concrete texture

# Patch 4 — bump patch (mesh) offset in +Y so the vehicle can be tested over a bump
patch4_mat = chrono.ChContactMaterialNSC()                         # fourth material
patch4_mat.SetFriction(0.9)
patch4_mat.SetRestitution(0.01)
patch4 = terrain.AddPatch(patch4_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(10, 25, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/meshes/bump.obj"))  # mesh bump obstacle
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 6, 6)  # tile texture on the bump

terrain.Initialize()                                               # build all four patches

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle Irrlicht visual system
vis.SetWindowTitle("Gator - multi-patch terrain (textures, grade, bump)")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera behind the chassis
vis.Initialize()                                                  # build device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                   # sky box
vis.AddLightDirectional()                                         # vehicle demos use a directional light
vis.AttachVehicle(gator.GetVehicle())                             # bind the vehicle visual assets

# Scripted open-loop maneuver: accelerate and drive straight +X across the patches and up the grade
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),                       # (time, steering, throttle, braking)
    veh.DataDriverEntry(1.0, 0.0, 0.0, 0.0),                       # settle on the grass patch
    veh.DataDriverEntry(2.0, 0.0, 0.8, 0.0),                       # accelerate forward
    veh.DataDriverEntry(12.0, 0.0, 1.0, 0.0),                      # full throttle to climb the grade
    veh.DataDriverEntry(16.0, 0.0, 1.0, 0.0),                      # hold to the end
])
driver = veh.ChDataDriver(gator.GetVehicle(), driver_data)        # scripted data driver
driver.Initialize()                                               # build the driver

render_step_size = 1.0 / 50.0                                     # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)           # physics steps per rendered frame
render_every = render_steps                                      # untagged cadence constant
realtime_timer = chrono.ChRealtimeStepTimer()                    # spin to wall-clock
step_number = 0                                                  # physics step counter


while vis.Run() and gator.GetSystem().GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = gator.GetSystem().GetChTime()
        driver_inputs = driver.GetInputs()                       # current scripted driver inputs

        driver.Synchronize(time)                                 # sync subsystems
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)                                # advance subsystems
        terrain.Advance(step_size)
        gator.Advance(step_size)                                 # advances the wrapper-owned system
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)                           # spin in place to match wall-clock
        if gator.GetSystem().GetChTime() >= sim_end:
            break
