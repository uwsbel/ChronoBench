import math
import pychrono.core     as chrono
import pychrono.irrlicht as irr          
import pychrono.vehicle  as veh





class SCMTerrainParameters:
    

    def __init__(self, preset: str = 'mid'):
        preset = preset.lower()

        if preset == 'soft':
            self.Kphi   = 1.0e6      
            self.Kc     = 0.0        
            self.n      = 1.1        
            self.c      = 0.0        
            self.phi    = 25.0       
            self.Janosi = 0.01       
            self.E      = 1.0e8      
            self.damp   = 2.0e4      

        elif preset == 'hard':
            self.Kphi   = 4.0e6
            self.Kc     = 0.0
            self.n      = 1.1
            self.c      = 0.0
            self.phi    = 35.0
            self.Janosi = 0.01
            self.E      = 4.0e8
            self.damp   = 4.0e4

        else:                     
            self.Kphi   = 2.0e6
            self.Kc     = 0.0
            self.n      = 1.1
            self.c      = 0.0
            self.phi    = 30.0
            self.Janosi = 0.01
            self.E      = 2.0e8
            self.damp   = 3.0e4

    
    def apply_to(self, scm_terrain: veh.SCMTerrain):
        
        scm_terrain.SetSoilParameters(self.Kphi,
                                      self.Kc,
                                      self.n,
                                      self.c,
                                      self.phi,
                                      self.Janosi,
                                      self.E,
                                      self.damp)





chrono.SetChronoDataPath(chrono.GetChronoDataPath())           
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')       





initLoc  = chrono.ChVector3d(-8, 0, 0.6)
initRot  = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type              = veh.VisualizationType.MESH
chassis_collision_type = veh.ChassisCollisionType.NONE
tire_model            = veh.TireModelType.RIGID

step_size        = 1e-3
tire_step_size   = step_size
render_step_size = 1.0 / 50.0                 





vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
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


soil = SCMTerrainParameters(preset='mid')     
soil.apply_to(terrain)                        


terrain.AddMovingPatch(vehicle.GetChassisBody(),
                       chrono.ChVector3d(0, 0, 0),
                       chrono.ChVector3d(5, 3, 1))

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)





trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV SCM-Terrain demo')
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
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size  / braking_time)
driver.Initialize()





print("===================================================")
print("  VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
print("  Using soil preset:", soil.__dict__)
print("===================================================")

render_steps     = math.ceil(render_step_size / step_size)
realtime_timer   = chrono.ChRealtimeStepTimer()
step_number      = 0
render_frame     = 0

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