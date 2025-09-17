import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


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


class SCMTerrainParameters:
    def __init__(self, kphi, kc, n, cohesive_limit, friction_limit, shear_coefficient, elastic_stiffness, damping):
        self.Bekker_Kphi = kphi
        self.Bekker_Kc = kc
        self.Bekker_n = n
        self.Mohr_cohesive_limit = cohesive_limit
        self.Mohr_friction_limit = friction_limit
        self.Janosi_shear_coefficient = shear_coefficient
        self.Elastic_stiffness = elastic_stiffness
        self.Damping = damping


def get_scm_parameters(config):
    if config == "soft":
        return SCMTerrainParameters(
            2e6, 
            0, 
            1.1, 
            0, 
            30, 
            0.01, 
            2e8, 
            3e4
        )
    elif config == "mid":
        return SCMTerrainParameters(
            4e6, 
            0, 
            1.1, 
            0, 
            30, 
            0.01, 
            4e8, 
            5e4
        )
    elif config == "hard":
        return SCMTerrainParameters(
            6e6, 
            0, 
            1.1, 
            0, 
            30, 
            0.01, 
            6e8, 
            7e4
        )
    else:
        raise ValueError("Invalid terrain configuration. Choose from 'soft', 'mid', or 'hard'.")



terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain_params = get_scm_parameters("mid")
terrain.SetSoilParameters(
    terrain_params.Bekker_Kphi,
    terrain_params.Bekker_Kc,
    terrain_params.Bekker_n,
    terrain_params.Mohr_cohesive_limit,
    terrain_params.Mohr_friction_limit,
    terrain_params.Janosi_shear_coefficient,
    terrain_params.Elastic_stiffness,
    terrain_params.Damping
)


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