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
    def __init__(self, Kphi=0, Kc=0, n=0, cohesion=0, friction=0, psi=0, E=0, nu=0):
        self.Kphi = Kphi  
        self.Kc = Kc      
        self.n = n        
        self.cohesion = cohesion  
        self.friction = friction  
        self.psi = psi    
        self.E = E        
        self.nu = nu      

    def SetSCMParameters(self, terrain, param_type):
        
        if param_type == "soft":
            self.Kphi = 0.5e6
            self.Kc = 0
            self.n = 1.1
            self.cohesion = 0
            self.friction = 20
            self.psi = 0.1
            self.E = 2e7
            self.nu = 3e4
        elif param_type == "mid":
            self.Kphi = 2e6
            self.Kc = 0
            self.n = 1.1
            self.cohesion = 0
            self.friction = 30
            self.psi = 0.03
            self.E = 2e8
            self.nu = 3e4
        elif param_type == "hard":
            self.Kphi = 53.1e6
            self.Kc = 1.5e6
            self.n = 1.37
            self.cohesion = 1.5e6
            self.friction = 31.0
            self.psi = 0.017
            self.E = 3.22e9
            self.nu = 2e5
        else:
            print(f"SCM parameter type [{param_type}] not recognized. Using default values.")
        terrain.SetSoilParameters(self.Kphi,   
                                self.Kc,     
                                self.n,      
                                self.cohesion,     
                                self.friction,    
                                self.psi,  
                                self.E,     
                                self.nu    
        )


terrain = veh.SCMTerrain(vehicle.GetSystem())
params = SCMTerrainParams()
params.SetSCMParameters(terrain, "mid")

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