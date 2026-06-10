import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                   # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # locate vehicle data files

step_size = 1e-3                   # simulation time step (s)
sim_end = 20.0                     # simulation duration (s)
render_fps = 50.0                  # render frames per second

terrainLength = 200.0              # terrain patch length (m)
terrainWidth = 200.0               # terrain patch width (m)

init_loc = chrono.ChVector3d(0, 0, 0.5)          # initial chassis position
init_rot = chrono.QuatFromAngleZ(0.0)            # initial rotation (no yaw)

bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)                       # NSC for rigid terrain
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)                                              # MANDATORY — fixed chassis won't move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_TMEASY)                              # TMEASY tires
bus.SetTireStepSize(step_size)
bus.Initialize()

system = bus.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # REQUIRED after Initialize

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)

print("VEHICLE MASS: ", bus.GetVehicle().GetMass())                    # truth's literal banner

terrain = veh.RigidTerrain(bus.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()                              # NSC contact material
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Data-driven driver with prescribed input sequence
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),                          # t=0.0: stationary
    veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0),                          # t=0.1: full throttle
    veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0),                          # t=0.5: steer + full throttle
])
driver = veh.ChDataDriver(bus.GetVehicle(), driver_data)               # data-driven driver (no time response)
driver.Initialize()

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus - Data Driver")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(bus.GetVehicle())

render_every = max(1, round(1.0 / (render_fps * step_size)))           # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
while vis.Run():
    time = bus.GetSystem().GetChTime()

    if step_number % render_every == 0:                                # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()


    driver.Synchronize(time)
    terrain.Synchronize(time)
    bus.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    bus.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)                                     # real-time pacing

    if time >= sim_end:
        break
