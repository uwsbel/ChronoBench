import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as irr





chrono.SetChronoDataPath("../data/")
chrono.SetVerbose(False)


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)






vehicle_file = "vehicle/Gator.json"
powertrain_file = "vehicle/Gator_SimplePowertrain.json"
tire_file = "vehicle/Gator_TMeasyTire.json"


init_loc = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)


gator = veh.WheeledVehicle(system, vehicle_file)
gator.Initialize(chrono.ChCoordsysD(init_loc, init_rot))
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)


powertrain = veh.SimplePowertrain(powertrain_file)
gator.InitializePowertrain(powertrain)


for axle in range(gator.GetNumberAxles()):
    for side in range(2):
        tire = veh.TMeasyTire(tire_file)
        gator.InitializeTire(tire, axle, veh.VehicleSide(side))





terrain = veh.RigidTerrain(system, "vehicle/terrain/RigidPlane.json")





driver = veh.ChIrrGuiDriver(gator.GetVehicle())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.1)
driver.Initialize()





vis_width = 1280
vis_height = 720
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(vis_width, vis_height)
vis.SetWindowTitle("Gator Vehicle Simulation with Sensors")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 4, 1.5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(10, 20, 30), chrono.ChVectorD(0, 0, 0), 150, 10, 40, 60, 512)






manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorD(0, 0, 100), chrono.ChVectorD(1, 1, 1), 1000)


chassis_body = gator.GetChassisBody()


offset_pose = chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 2),  
    chrono.Q_from_AngZ(0)  
)
camera = sens.ChCameraSensor(
    chassis_body,          
    30.0,                  
    offset_pose,           
    vis_width,             
    vis_height,            
    chrono.CH_C_PI / 3     
)
camera.SetName("Camera Sensor")
camera.SetLag(0.1)         
camera.SetCollectionWindow(0.05)  


camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterSave("camera/image_"))


manager.AddSensor(camera)






step_size = 1e-3
time_end = 20.0


output_step_size = 0.1
gator.GetVehicle().SetChassisOutput(True)
gator.GetVehicle().SetSuspensionOutput(0, True)
gator.GetVehicle().SetSteeringOutput(0, True)


realtime_timer = chrono.ChRealtimeStepTimer()
frame_count = 0

while vis.Run() and system.GetChTime() < time_end:
    
    driver_inputs = driver.GetInputs()
    
    
    driver.Synchronize(system.GetChTime())
    terrain.Synchronize(system.GetChTime())
    gator.Synchronize(system.GetChTime(), driver_inputs, terrain)
    manager.Update()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    system.DoStepDynamics(step_size)
    
    
    realtime_timer.Spin(step_size)
    
    
    if frame_count % 100 == 0:
        speed_kmh = gator.GetVehicle().GetSpeed() * 3.6
        print(f"Time: {system.GetChTime():.1f} s, Speed: {speed_kmh:.1f} km/h")
    
    frame_count += 1


vis.GetDevice().closeDevice()