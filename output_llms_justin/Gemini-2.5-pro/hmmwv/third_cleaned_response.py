import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math



chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


class MyDriver(veh.ChDriver):
    def __init__(self, vehicle_object, delay):
        
        super().__init__(vehicle_object)
        self.delay = delay
        
        self.SetThrottle(0.0)
        self.SetSteering(0.0)
        self.SetBraking(0.0)

    def Synchronize(self, time):
        
        effective_time = time - self.delay

        current_throttle = 0.0
        current_steering = 0.0
        current_braking = 0.0  

        if effective_time >= 0: 
            
            
            
            if effective_time >= 0.2:
                current_throttle = 0.7
            

            
            
            
            if effective_time >= 2.0:
                
                current_steering = 0.5 * math.sin(2 * math.pi * 1.0 * (effective_time - 2.0))
            
        
        
        
        
        self.SetThrottle(current_throttle)
        self.SetSteering(current_steering)
        self.SetBraking(current_braking)




initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0) 


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE 


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0.0    
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC


step_size = 1e-3
tire_step_size = step_size 


render_step_size = 1.0 / 50  


simulation_end_time = 4.0




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


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())

patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT), 
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Custom Driver Demo') 
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5) 
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())




driver_delay = 0.5  
driver = MyDriver(vehicle.GetVehicle(), driver_delay)




print(f"VEHICLE MASS: {vehicle.GetVehicle().GetMass()}")




render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


while vis.Run(): 
    time = vehicle.GetSystem().GetChTime()

    
    if time >= simulation_end_time:
        print(f"Simulation time {time:.2f}s has reached the limit of {simulation_end_time:.2f}s. Ending simulation.")
        break

    
    if (step_number % render_steps == 0):
        vis.BeginScene(True, True, chrono.ChColor(0.3,0.3,0.3)) 
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver.Synchronize(time)    
    terrain.Synchronize(time)
    
    vehicle.Synchronize(time, driver, terrain) 
    
    vis.Synchronize(time, driver.GetInputs())

    
    driver.Advance(step_size)   
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)      

    
    step_number += 1

    
    realtime_timer.Spin(step_size)