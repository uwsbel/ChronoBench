import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                    # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')                # locate vehicle data files

step_size = 2e-3                                                         # integration step (s)
init_loc = chrono.ChVector3d(-30.0, 0.0, 0.5)                           # spawn on the first flat patch
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                            # facing +X

gator = veh.Gator()                                                     # Gator catalog vehicle
gator.SetContactMethod(chrono.ChContactMethod_NSC)                     # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)                 # chassis collision off
gator.SetChassisFixed(False)                                           # MANDATORY — fixed chassis won't move
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # initial pose
gator.SetTireType(veh.TireModelType_TMEASY)                           # TMEASY tires roll over heightmap/bump
gator.SetTireStepSize(step_size)                                       # tire integration step
gator.Initialize()                                                     # build the vehicle subsystems

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)      # mesh chassis
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)        # mesh wheels
gator.SetTireVisualizationType(veh.VisualizationType_MESH)         # mesh tires

system = gator.GetSystem()                                             # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())                 # report total vehicle mass

terrain = veh.RigidTerrain(system)                                     # rigid multi-patch terrain

patch_mat = chrono.ChContactMaterialNSC()                             # shared NSC patch material
patch_mat.SetFriction(0.9)                                             # high grip for gradability
patch_mat.SetRestitution(0.01)                                         # nearly inelastic

# Patch 1 — flat grass patch (spawn / run-up), centered at x=-30
patch1 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(-30, 0, 0), chrono.QUNIT), 30, 20)
patch1.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)  # grass texture

# Patch 2 — flat concrete patch, centered at x=0
patch2 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 30, 20)
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 16, 16)  # concrete texture

# Patch 3 — bump patch (heightmap bump the vehicle drives over), centered at x=30
patch3 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(30, 0, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/height_maps/bump64.bmp"), 30, 20, 0.0, 0.6)  # bump rise 0..0.6 m
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 16, 16)  # tile texture

# Patch 4 — heightmap gradability slope patch (gradability ramp), centered at x=60
patch4 = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(60, 0, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/height_maps/slope.bmp"), 30, 20, 0.0, 4.0)  # heightmap slope, 0..4 m rise
patch4.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 16, 16)  # dirt texture

terrain.Initialize()                                                   # finalize terrain bodies

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                      # vehicle Irrlicht visualizer
vis.SetWindowTitle("Gator Multi-Patch Terrain")                       # window title
vis.SetWindowSize(1280, 720)                                          # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.5)          # chase camera behind chassis
vis.Initialize()                                                      # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo
vis.AddSkyBox()                                                       # sky box
vis.AddLightDirectional()                                            # vehicle demos use a directional light
vis.AttachVehicle(gator.GetVehicle())                                # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive driver (truth default)
render_step_size = 1.0 / 50.0                                        # render cadence
driver.SetSteeringDelta(render_step_size / 1.0)                     # steering ramp rate
driver.SetThrottleDelta(render_step_size / 1.0)                     # throttle ramp rate
driver.SetBrakingDelta(render_step_size / 0.3)                     # brake ramp rate
driver.Initialize()                                                 # build the driver

render_every = max(1, round(1.0 / (50.0 * step_size)))             # untagged render-cadence constant
sim_end = 30.0                                                      # total sim time (s)
realtime_timer = chrono.ChRealtimeStepTimer()                      # wall-clock pacer

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                               # begin frame
    vis.Render()                                                   # draw scene
    vis.EndScene()                                                 # present frame
    for _ in range(render_every):
        time = system.GetChTime()                                  # current sim time
        driver.Synchronize(time)                                  # update driver
        driver_inputs = driver.GetInputs()                        # current driver inputs
        driver_inputs.m_throttle = 0.7                            # drive forward to climb the slope / bump
        driver_inputs.m_steering = 0.0                            # straight ahead
        driver_inputs.m_braking = 0.0                             # no brake

        terrain.Synchronize(time)                                 # update terrain
        gator.Synchronize(time, driver_inputs, terrain)          # update vehicle with terrain
        vis.Synchronize(time, driver_inputs)                     # update visuals


        driver.Advance(step_size)                                 # advance driver
        terrain.Advance(step_size)                                # advance terrain
        gator.Advance(step_size)                                  # advance the wrapper-owned system
        vis.Advance(step_size)                                    # advance visuals
        realtime_timer.Spin(step_size)                           # pace to wall clock
        if system.GetChTime() >= sim_end:
            break
