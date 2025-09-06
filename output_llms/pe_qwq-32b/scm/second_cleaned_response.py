import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


class SCMTerrainParameters:
    def __init__(self, config="mid"):
        self.config = config
        self.params = {}
        if config == "soft":
            self.params = {
                'Kphi': 2e6,
                'Kc': 0,
                'n': 1.1,
                'cohesion': 0,
                'friction': 30,
                'shear_coeff': 0.01,
                'elastic_stiffness': 2e8,
                'damping': 3e4
            }
        elif config == "mid":
            self.params = {
                'Kphi': 5e6,
                'Kc': 0,
                'n': 1.0,
                'cohesion': 100,
                'friction': 35,
                'shear_coeff': 0.005,
                'elastic_stiffness': 3e8,
                'damping': 2e4
            }
        elif config == "hard":
            self.params = {
                'Kphi': 1e7,
                'Kc': 0,
                'n': 0.9,
                'cohesion': 200,
                'friction': 40,
                'shear_coeff': 0.001,
                'elastic_stiffness': 5e8,
                'damping': 1e4
            }
        else:
            raise ValueError("Invalid terrain configuration")

    def apply_to_terrain(self, terrain):
        terrain.SetSoilParameters(
            self.params['Kphi'],
            self.params['Kc'],
            self.params['n'],
            self.params['cohesion'],
            self.params['friction'],
            self.params['shear_coeff'],
            self.params['elastic_stiffness'],
            self.params['damping']
        )

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(-8, 0, 0.6)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVectorD(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
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
terrain_config = "mid"  
terrain_params = SCMTerrainParameters(terrain_config)
terrain_params.apply_to_terrain(terrain)


terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(terrainLength, terrainWidth, 0.02)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

print(f"VEHICLE MASS: {vehicle.GetMass()}")

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