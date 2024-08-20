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


vehicle = veh.HMMWV_Full() 
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)


terrain_params = TerrainParameters()
terrain_params.set_parameters("soft")

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.set_parameters(terrain_params)


terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))


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

class TerrainParameters:
    def __init__(self):
        self.bekker_kphi = 2e6
        self.bekker_kc = 0
        self.bekker_n_exponent = 1.1
        self.mohr_cohesive_limit = 0
        self.mohr_friction_limit = 30
        self.janosi_shear_coefficient = 0.01
        self.elastic_stiffness = 2e8
        self.damping = 3e4

    def set_parameters(self, config):
        if config == "soft":
            self.bekker_kphi = 1e6
            self.bekker_kc = 1e3
            self.bekker_n_exponent = 1.2
            self.mohr_cohesive_limit = 1e3
            self.mohr_friction_limit = 20
            self.janosi_shear_coefficient = 0.05
            self.elastic_stiffness = 1e8
            self.damping = 1e4
        elif config == "mid":
            self.bekker_kphi = 2e6
            self.bekker_kc = 1e3
            self.bekker_n_exponent = 1.1
            self.mohr_cohesive_limit = 1e3
            self.mohr_friction_limit = 25
            self.janosi_shear_coefficient = 0.01
            self.elastic_stiffness = 2e8
            self.damping = 3e4
        elif config == "hard":
            self.bekker_kphi = 3e6
            self.bekker_kc = 1e4
            self.bekker_n_exponent = 1.0
            self.mohr_cohesive_limit = 1e4
            self.mohr_friction_limit = 30
            self.janosi_shear_coefficient = 0.005
            self.elastic_stiffness = 3e8
            self.damping = 5e4

    def get_parameters(self):
        return {
            "bekker_kphi": self.bekker_kphi,
            "bekker_kc": self.bekker_kc,
            "bekker_n_exponent": self.bekker_n_exponent,
            "mohr_cohesive_limit": self.mohr_cohesive_limit,
            "mohr_friction_limit": self.mohr_friction_limit,
            "janosi_shear_coefficient": self.janosi_shear_coefficient,
            "elastic_stiffness": self.elastic_stiffness,
            "damping": self.damping
        }

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