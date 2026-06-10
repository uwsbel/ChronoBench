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
    

    
    PRESETS = {
        'soft': {
            'Kphi': 0.2e6,
            'Kc': 0,
            'n': 1.1,
            'cohesion': 0,
            'friction': 20,
            'Janosi': 0.01,
            'elastic_stiffness': 2e7,
            'damping': 3e4,
        },
        'mid': {
            'Kphi': 2e6,
            'Kc': 0,
            'n': 1.1,
            'cohesion': 0,
            'friction': 30,
            'Janosi': 0.01,
            'elastic_stiffness': 2e8,
            'damping': 3e4,
        },
        'hard': {
            'Kphi': 5e6,
            'Kc': 0,
            'n': 1.1,
            'cohesion': 0,
            'friction': 40,
            'Janosi': 0.01,
            'elastic_stiffness': 2e9,
            'damping': 3e4,
        },
    }

    def __init__(self, preset='mid'):
        
        if preset not in self.PRESETS:
            raise ValueError(f"Unknown preset '{preset}'. Choose from: {list(self.PRESETS.keys())}")

        params = self.PRESETS[preset]
        self.Kphi = params['Kphi']                       
        self.Kc = params['Kc']                           
        self.n = params['n']                             
        self.cohesion = params['cohesion']               
        self.friction = params['friction']               
        self.Janosi = params['Janosi']                   
        self.elastic_stiffness = params['elastic_stiffness']  
        self.damping = params['damping']                 

        print(f"SCM Terrain initialized with '{preset}' preset:")
        print(f"  Bekker Kphi          : {self.Kphi}")
        print(f"  Bekker Kc            : {self.Kc}")
        print(f"  Bekker n             : {self.n}")
        print(f"  Cohesion             : {self.cohesion} Pa")
        print(f"  Friction             : {self.friction} degrees")
        print(f"  Janosi shear coeff   : {self.Janosi} m")
        print(f"  Elastic stiffness    : {self.elastic_stiffness} Pa/m")
        print(f"  Damping              : {self.damping} Pa s/m")

    def ApplyToTerrain(self, terrain):
        
        terrain.SetSoilParameters(
            self.Kphi,
            self.Kc,
            self.n,
            self.cohesion,
            self.friction,
            self.Janosi,
            self.elastic_stiffness,
            self.damping
        )





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



scm_params = SCMTerrainParameters(preset='mid')
scm_params.ApplyToTerrain(terrain)


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

    
    if step_number % render_steps == 0:
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

scm_params = SCMTerrainParameters(preset='mid')
   scm_params.ApplyToTerrain(terrain)