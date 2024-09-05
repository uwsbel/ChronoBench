import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


class SCMParameters:
    def __init__(self, config):
        if config == "soft":
            self.bekker_Kphi = 2e5
            self.bekker_Kc = 0
            self.bekker_n = 1.1
            self.mohr_cohesive_limit = 0
            self.mohr_friction_limit = 30
            self.janosi_shear_coefficient = 0.01
            self.elastic_stiffness = 2e7
            self.damping = 3e3
        elif config == "mid":
            self.bekker_Kphi = 2e6
            self.bekker_Kc = 0
            self.bekker_n = 1.1
            self.mohr_cohesive_limit = 0
            self.mohr_friction_limit = 30
            self.janosi_shear_coefficient = 0.01
            self.elastic_stiffness = 2e8
            self.damping = 3e4
        elif config == "hard":
            self.bekker_Kphi = 2e7
            self.bekker_Kc = 0
            self.bekker_n = 1.1
            self.mohr_cohesive_limit = 0
            self.mohr_friction_limit = 30
            self.janosi_shear_coefficient = 0.01
            self.elastic_stiffness = 2e9
            self.damping = 3e5
        else:
            raise ValueError("Invalid configuration. Choose from 'soft', 'mid', or 'hard'.")


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



vehicle = veh.HMMWV_Full()  
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.SCMTerrain(vehicle.GetSystem())


scm_params = SCMParameters("mid")
terrain.SetSoilParameters(scm_params.bekker_Kphi,
                          scm_params.bekker_Kc,
                          scm_params.bekker_n,
                          scm_params.mohr_cohesive_limit,
                          scm_params.mohr_friction_limit,
                          scm_params.janosi_shear_coefficient,
                          scm_params.elastic_stiffness,
                          scm_params.damping)


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






print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
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