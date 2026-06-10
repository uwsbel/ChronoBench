import os
import math

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr





ChVector = chrono.ChVector3d if hasattr(chrono, "ChVector3d") else chrono.ChVectorD
ChQuaternion = chrono.ChQuaterniond if hasattr(chrono, "ChQuaterniond") else chrono.ChQuaternionD
ChCoordsys = chrono.ChCoordsysd if hasattr(chrono, "ChCoordsysd") else chrono.ChCoordsysD





if "CHRONO_DATA_DIR" in os.environ:
    chrono.SetChronoDataPath(os.environ["CHRONO_DATA_DIR"])

veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), "vehicle") + os.sep)





step_size = 2.0e-3
tire_step_size = 1.0e-3

render_fps = 50
render_step_size = 1.0 / render_fps
render_steps = int(math.ceil(render_step_size / step_size))


init_location = ChVector(0.0, 0.0, 1.0)
init_orientation = ChQuaternion(1.0, 0.0, 0.0, 0.0)


terrain_length = 100.0
terrain_width = 20.0
terrain_delta = 0.05


moving_patch_center = ChVector(0.0, 0.0, 0.0)
moving_patch_size = ChVector(6.0, 4.0, 1.0)


bekker_Kphi = 2.0e6       
bekker_Kc = 0.0           
bekker_n = 1.1            
mohr_cohesion = 0.0       
mohr_friction = 30.0      
janosi_shear = 0.01       
elastic_K = 4.0e7         
damping_R = 3.0e4         


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3





hmmwv = veh.HMMWV_Full()

hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)

hmmwv.SetInitPosition(ChCoordsys(init_location, init_orientation))

hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)


hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(tire_step_size)

hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


system = hmmwv.GetSystem()
system.SetGravitationalAcceleration(ChVector(0.0, 0.0, -9.81))





terrain = veh.SCMTerrain(system)

terrain.SetSoilParameters(
    bekker_Kphi,
    bekker_Kc,
    bekker_n,
    mohr_cohesion,
    mohr_friction,
    janosi_shear,
    elastic_K,
    damping_R,
)

terrain.Initialize(terrain_length, terrain_width, terrain_delta)


terrain.AddMovingPatch(
    hmmwv.GetChassisBody(),
    moving_patch_center,
    moving_patch_size,
)


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.20)





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono HMMWV on SCM Deformable Terrain")
vis.SetWindowSize(1280, 720)


vis.SetChaseCamera(ChVector(0.0, 0.0, 1.75), 8.0, 0.5)

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())





driver = veh.ChInteractiveDriverIRR(vis)

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()





realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = system.GetChTime()

    
    if step_number % render_steps == 0:
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

    
    realtime_timer.Spin(step_size)

    step_number += 1