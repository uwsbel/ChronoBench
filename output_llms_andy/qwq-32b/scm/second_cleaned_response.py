import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


class SoilParameters:
    def __init__(self, config):
        self.config = config
        self.kphi = 0.0
        self.kc = 0.0
        self.n = 0.0
        self.cohesive = 0.0
        self.friction = 0.0
        self.shear = 0.0
        self.elastic = 0.0
        self.damping = 0.0
        self.set_parameters(config)
    
    def set_parameters(self, config):
        if config == 'soft':
            self.kphi = 2e6
            self.kc = 0
            self.n = 1.1
            self.cohesive = 0
            self.friction = 30
            self.shear = 0.01
            self.elastic = 2e8
            self.damping = 3e4
        elif config == 'mid':
            self.kphi = 5e6
            self.kc = 0
            self.n = 1.2
            self.cohesive = 0
            self.friction = 35
            self.shear = 0.005
            self.elastic = 5e8
            self.damping = 2e4
        elif config == 'hard':
            self.kphi = 1e7
            self.kc = 0
            self.n = 1.3
            self.cohesive = 0
            self.friction = 40
            self.shear = 0.001
            self.elastic = 1e9
            self.damping = 1e4
        else:
            raise ValueError("Invalid configuration")

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


soil_config = 'soft'  
soil_params = SoilParameters(soil_config)

terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    soil_params.kphi,
    soil_params.kc,
    soil_params.n,
    soil_params.cohesive,
    soil_params.friction,
    soil_params.shear,
    soil_params.elastic,
    soil_params.damping
)


terrain.Initialize(20, 20, 1.0, 100, 100, 10)  

terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


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
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if step_number % math.ceil(render_step_size / step_size) == 0:
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