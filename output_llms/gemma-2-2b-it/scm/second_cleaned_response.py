import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


class SCMTerrainParameters:
    def __init__(self, soil_parameters, soil_parameters_2, soil_parameters_3, soil_parameters_4, soil_parameters_5, soil_parameters_6, soil_parameters_7, soil_parameters_8):
        self.soil_parameters = soil_parameters
        self.soil_parameters_2 = soil_parameters_2
        self.soil_parameters_3 = soil_parameters_3
        self.soil_parameters_4 = soil_parameters_4
        self.soil_parameters_5 = soil_parameters_5
        self.soil_parameters_6 = soil_parameters_6
        self.soil_parameters_7 = soil_parameters_7
        self.soil_parameters_8 = soil_parameters_8


def InitializeTerrainParameters(terrain_type):
    if terrain_type == "soft":
        soil_parameters = (2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)  
    elif terrain_type == "mid":
        soil_parameters = (1e6, 0, 1.5, 0, 40, 0.02, 1e7, 5e4)
    elif terrain_type == "hard":
        soil_parameters = (5e6, 0, 1.9, 0, 50, 0.05, 3e8, 7e4)
    else:
        raise ValueError("Invalid terrain type. Choose from 'soft', 'mid', or 'hard'.")

    return SCMTerrainParameters(soil_parameters)


initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID



terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  



vehicle = veh.HMMWV_Full() 
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(InitializeTerrainParameters("soft"))




terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.Initialize(20, 20, 0.02)



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())



driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)