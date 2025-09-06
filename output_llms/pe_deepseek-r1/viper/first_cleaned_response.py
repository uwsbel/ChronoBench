import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math




system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))




terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.CSYSNORM, 
                         100, 100)  
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
terrain.Initialize()




rover = veh.Viper(system)
rover.SetDriverPos(chrono.ChVector3d(0, 0, 0.5))  
rover.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), 
                                   chrono.QuatFromAngleY(0)))


driver = veh.ViperSpeedDriver()
driver.SetThrottle(0.3)  
rover.SetDriver(driver)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 3, 1.5), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(5, 5, 5), chrono.ChVector3d(0, 0, 0.5), 
                       10, 2, 10, 40, 512, chrono.ChColor(0.8, 0.8, 0.8))


rover.InitializeVisualization(vis)




step_size = 0.001
end_time = 30.0  
steering_period = 10.0  
max_steering = 0.5  




time = 0.0
while time < end_time:
    
    steering = max_steering * math.sin(2 * chrono.CH_PI * time / steering_period)
    driver.SetSteering(steering)
    
    
    rover.Update()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(step_size)
    time += step_size

    
    pos = rover.GetChassis().GetPos()
    speed = rover.GetSpeed()
    vis.GetGUIEnvironment().addStaticText(
        f"Time: {time:.2f}s | Steering: {steering:.2f} | Speed: {speed:.2f} m/s | Position: ({pos.x:.2f}, {pos.y:.2f})",
        chronoirr.recti(10, 10, 400, 40))


   terrain.EnableMeshCreation(True)
   terrain.SetMeshResolution(0.5)  


   rover.SetWheelContactMaterial(chrono.ChContactMaterialSMC())
   rover.SetChassisVisualizationType(veh.VisualizationType_MESH)


   steering = max_steering * min(time / 5.0, 1.0)  


   vis.CameraFollow(rover.GetChassisBody(), chrono.ChVector3d(0, 5, 2), 5.0)