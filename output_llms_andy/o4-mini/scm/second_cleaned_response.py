import math

import pychrono.core as chrono
import pychrono.irrlicht as irr  
import pychrono.vehicle as veh




class SCMTerrainParams:
    
    def __init__(self, preset="mid"):
        if preset == "soft":
            self.Kphi = 1e6       
            self.Kc   = 0         
            self.n    = 1.0       
            self.coh  = 0.0       
            self.phi  = 20.0      
            self.J     = 0.005    
            self.E     = 1e8      
            self.damp  = 1e3      
        elif preset == "hard":
            self.Kphi = 4e6
            self.Kc   = 0
            self.n    = 1.2
            self.coh  = 0.0
            self.phi  = 35.0
            self.J     = 0.02
            self.E     = 5e8
            self.damp  = 5e4
        else:
            
            self.Kphi = 2e6
            self.Kc   = 0
            self.n    = 1.1
            self.coh  = 0.0
            self.phi  = 30.0
            self.J     = 0.01
            self.E     = 2e8
            self.damp  = 3e4

    def apply(self, terrain):
        
        terrain.SetSoilParameters(
            self.Kphi,   
            self.Kc,     
            self.n,      
            self.coh,    
            self.phi,    
            self.J,      
            self.E,      
            self.damp    
        )






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model             = veh.TireModelType_RIGID


contact_method = chrono.ChContactMethod_SMC


step_size        = 1e-3
tire_step_size   = step_size
render_fps       = 50
render_step_size = 1.0 / render_fps
render_steps     = math.ceil(render_step_size / step_size)


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


soil_choice = "mid"
soil_params = SCMTerrainParams(soil_choice)
soil_params.apply(terrain)


terrain.AddMovingPatch(
    vehicle.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1)
)


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.Initialize(20, 20, 0.02)




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV SCMTerrain Demo')
vis.SetWindowSize(1280, 1024)

trackPoint = chrono.ChVector3d(0, 0, 1.71)
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


print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())




realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
render_frame   = 0

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