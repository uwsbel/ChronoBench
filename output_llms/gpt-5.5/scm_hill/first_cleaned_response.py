import os
import math

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr









chrono_data = chrono.GetChronoDataPath()
veh.SetDataPath(os.path.join(chrono_data, "vehicle") + os.sep)






step_size = 2.0e-3
tire_step_size = 1.0e-3
render_step_size = 1.0 / 50.0

init_location = chrono.ChVector3d(-25.0, 0.0, 0.85)
init_rotation = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
init_fwd_vel = 0.0

terrain_length = 80.0
terrain_width = 30.0
terrain_delta = 0.05

height_map_file = veh.GetDataFile("terrain/height_maps/bump64.bmp")
height_min = -0.05
height_max = 0.10






hmmwv = veh.HMMWV_Full()

hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_location, init_rotation))
hmmwv.SetInitFwdVel(init_fwd_vel)

hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)


hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(tire_step_size)

hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()






terrain = veh.SCMTerrain(system)






terrain.SetSoilParameters(
    2.0e6,   
    0.0,     
    1.1,     
    0.0,     
    30.0,    
    0.01,    
    4.0e7,   
    3.0e4    
)

terrain.Initialize(
    height_map_file,
    terrain_length,
    terrain_width,
    height_min,
    height_max,
    terrain_delta
)


try:
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.15)
except Exception:
    pass






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.AttachVehicle(hmmwv.GetVehicle())
vis.SetWindowTitle("PyChrono HMMWV on SCM Height-Map Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 7.0, 0.5)

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()






driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = system.GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)