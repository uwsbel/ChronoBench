import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr




TIME_STEP = 0.02  
FPS = 50           
REALTIME_FACTOR = 1.0  


INIT_POS = chrono.ChVector3d(0, 0.5, 0)  
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)  
CONTACT_METHOD = chrono.ChContactMethod_SMC  
TIRE_MODEL = veh.TireModelType_TMEASY       
VISUALIZATION_TYPE = veh.VisualizationType_MESH  


TERRAIN_LENGTH = 100.0  
TERRAIN_WIDTH = 10.0    
TEXTURE_FILE = "terrain_texture.jpg"  





system = chrono.ChSystemSMC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravity(chrono.ChVector3d(0, -9.81, 0))


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))





gator = veh.Gator()
gator.SetContactMethod(CONTACT_METHOD)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
gator.SetTireType(TIRE_MODEL)
gator.SetTireStepSize(TIME_STEP)
gator.Initialize()


gator.SetChassisVisualizationType(VISUALIZATION_TYPE)
gator.SetSuspensionVisualizationType(VISUALIZATION_TYPE)
gator.SetSteeringVisualizationType(VISUALIZATION_TYPE)
gator.SetWheelVisualizationType(VISUALIZATION_TYPE)
gator.SetTireVisualizationType(VISUALIZATION_TYPE)


vehicle = gator.GetVehicle()
tires = gator.GetTires()





terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)

patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), 
                         chrono.QuatFromAngleX(0)), 
                         TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile(TEXTURE_FILE), TERRAIN_LENGTH, TERRAIN_WIDTH)
terrain.Initialize()





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -10))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(5, 5, 5), chrono.ChVector3d(0, 0, 0), 50, 1, 50, 50, 512)


gator.SetChaseCamera(chrono.ChVector3d(0, 0, 0), 6.0, 0.5)
gator.GetVehicle().Synchronize(0.0)
gator.GetVehicle().Advance(TIME_STEP)





driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()





frame_interval = 1.0 / FPS
last_frame_time = 0
time = 0

while vis.Run():
    
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    
    
    gator.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    
    system.DoStepDynamics(TIME_STEP)
    
    
    if time - last_frame_time >= frame_interval:
        vis.BeginScene()
        vis.Render()
        vis.RenderFrame(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0), 1.0)
        vis.EndScene()
        last_frame_time = time
        
        
        vis.SetCameraTarget(gator.GetChassisBody().GetPos())
        vis.GetActiveCamera().SetAngle(50)
        
        
        speed = vehicle.GetSpeed() * 3.6
        vis.ShowInfoPanel(True)
        vis.AddToInfoPanel("Vehicle Speed: ", f"{speed:.2f} km/h")
        vis.AddToInfoPanel("Throttle: ", f"{driver_inputs.m_throttle*100:.1f}%")
        vis.AddToInfoPanel("Braking: ", f"{driver_inputs.m_braking*100:.1f}%")
        vis.AddToInfoPanel("Steering: ", f"{driver_inputs.m_steering*100:.1f}%")
    
    
    chrono.ChRealtimeStep(TIME_STEP * REALTIME_FACTOR)

print("Simulation completed successfully.")