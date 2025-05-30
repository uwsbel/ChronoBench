import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


class SCMTerrainParameters:
    def __init__(self, config="mid"):
        self.config = config
        self.parameters = self.get_parameters(config)

    def get_parameters(self, config):
        
        configurations = {
            "soft": {
                "Bekker_Kphi": 1e6,
                "Bekker_Kc": 0,
                "Bekker_n": 1.0,
                "Mohr_cohesion": 0,
                "Mohr_friction_angle": 20,
                "Janosi_shear": 0.01,
                "elastic_stiffness": 1e7,
                "damping": 3e4
            },
            "mid": {
                "Bekker_Kphi": 2e6,
                "Bekker_Kc": 0,
                "Bekker_n": 1.1,
                "Mohr_cohesion": 0,
                "Mohr_friction_angle": 30,
                "Janosi_shear": 0.01,
                "elastic_stiffness": 2e8,
                "damping": 3e4
            },
            "hard": {
                "Bekker_Kphi": 3e6,
                "Bekker_Kc": 0,
                "Bekker_n": 1.2,
                "Mohr_cohesion": 0,
                "Mohr_friction_angle": 40,
                "Janosi_shear": 0.01,
                "elastic_stiffness": 3e8,
                "damping": 3e4
            }
        }
        return configurations.get(config, configurations["mid"])

    def set_terrain_parameters(self, terrain):
        params = self.parameters
        terrain.SetSoilParameters(params["Bekker_Kphi"], params["Bekker_Kc"], params["Bekker_n"],
                                  params["Mohr_cohesion"], params["Mohr_friction_angle"], params["Janosi_shear"],
                                  params["elastic_stiffness"], params["damping"])


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

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


terrain_params = SCMTerrainParameters("mid")
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain_params.set_terrain_parameters(terrain)

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
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


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

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())