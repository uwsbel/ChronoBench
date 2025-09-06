import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.driver as chronodriv


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", "data"))


sys = chrono.ChSystemSMC()


sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         200, 100)  
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()


car = veh.WheeledVehicle(sys, chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv.json"))
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QUNIT))
car.SetTireType(veh.TireModelType_TMEASY)  
car.SetChassisCollisionType(veh.ChassisCollisionType_ALL)  
car.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('BMW E90 Sedan Simulation - TMEASY Tires on Rigid Terrain')
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, -5))  
vis.AddTypicalLights(chrono.ChVector3d(30, 50, 30),  
                     chrono.ChColor(0.7, 0.7, 0.7),  
                     chrono.ChColor(0.9, 0.9, 0.9))  


driver = chronodriv.ChIrrGuiDriver(vis.GetDevice())
driver.SetSteeringDelta(0.05)   
driver.SetThrottleDelta(0.05)  
driver.SetBrakingDelta(0.05)   
driver.Initialize()


time_step = 0.01  

while vis.Run():
    
    time = sys.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    car.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    
    sys.DoStepDynamics(time_step)
    
    
    chassis = car.GetChassis()
    chassis_pos = chassis.GetPos()
    chassis_rot = chassis.GetRot()
    
    
    chase_offset = chrono.ChVector3d(0, 1.5, -4)
    chase_offset = chassis_rot.Rotate(chase_offset)
    
    cam_pos = chassis_pos + chase_offset
    cam_target = chassis_pos + chrono.ChVector3d(0, 0.5, 0)
    
    vis.SetCameraPosition(cam_pos, cam_target)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()