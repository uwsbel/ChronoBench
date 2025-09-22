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

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


class SCMTerrainParams:
    def __init__(self):
        self.Bekker_Kphi = 2e6
        self.Bekker_Kc = 0
        self.Bekker_n = 1.1
        self.Mohr_cohesive_limit = 0
        self.Mohr_friction_limit = 30
        self.Janosi_shear_coeff = 0.01
        self.Elastic_stiffness = 2e8
        self.Damping = 3e4
        self.plot_type = veh.SCMTerrain.PLOT_SINKAGE
        self.plot_color = 0.1

    def SetBekkerKphi(self, value):
        self.Bekker_Kphi = value

    def SetBekkerKc(self, value):
        self.Bekker_Kc = value

    def SetBekkerN(self, value):
        self.Bekker_n = value

    def SetMohrCohesiveLimit(self, value):
        self.Mohr_cohesive_limit = value

    def SetMohrFrictionLimit(self, value):
        self.Mohr_friction_limit = value

    def SetJanosiShearCoeff(self, value):
        self.Janosi_shear_coeff = value

    def SetElasticStiffness(self, value):
        self.Elastic_stiffness = value

    def SetDamping(self, value):
        self.Damping = value

    def SetPlotType(self, plot_type):
        self.plot_type = plot_type

    def SetPlotColor(self, color):
        self.plot_color = color

terrain_params = SCMTerrainParams()
terrain_params.SetBekkerKphi(2e6)
terrain_params.SetBekkerKc(0)
terrain_params.SetBekkerN(1.1)
terrain_params.SetMohrCohesiveLimit(0)
terrain_params.SetMohrFrictionLimit(30)
terrain_params.SetJanosiShearCoeff(0.01)
terrain_params.SetElasticStiffness(2e8)
terrain_params.SetDamping(3e4)
terrain_params.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain_params.SetPlotColor(0.1)


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    terrain_params.Bekker_Kphi,
    terrain_params.Bekker_Kc,
    terrain_params.Bekker_n,
    terrain_params.Mohr_cohesive_limit,
    terrain_params.Mohr_friction_limit,
    terrain_params.Janosi_shear_coeff,
    terrain_params.Elastic_stiffness,
    terrain_params.Damping
)
terrain.SetPlotType(terrain_params.plot_type, terrain_params.plot_color)


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