import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("data/")
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))



terrain = chrono.ChRigidBody()
terrain.SetBodyFixed(True)
terrain_shape = chrono.ChBoxShape()
terrain_shape.SetLengths(chrono.ChVectorD(100, 1, 100))
terrain.AddShape(terrain_shape)
my_system.Add(terrain)


gator = chrono.ChVehicle("gator")
gator.SetChassisBody(chrono.ChBodyEasyBox(2, 1, 4, 1000))
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChVectorD(0, 1, 0))
my_system.Add(gator.GetChassisBody())


gator.GetChassisBody().SetCollide(True)
gator.GetChassisBody().SetBodyFixed(False)
gator.GetChassisBody().AddAsset(chrono.ChAssetLevel(chrono.ChColorAsset(0.8, 0.2, 0.2)))



driver = chrono.ChDriver(gator)
driver.SetSteeringAngle(0)
driver.SetThrottle(0)
driver.SetBraking(0)


sensor_manager = chrono.ChSensorManager(my_system)


light1 = chrono.ChLightPoint()
light1.SetPosition(chrono.ChVectorD(5, 5, 5))
light1.SetIntensity(1000)
sensor_manager.AddLight(light1)


camera = chrono.ChCamera()
camera.SetPosition(chrono.ChVectorD(0, 3, 5))
camera.SetLookAt(chrono.ChVectorD(0, 1, 0))
camera.SetVertical(chrono.ChVectorD(0, 1, 0))
sensor_manager.AddCamera(camera)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Gator Simulation")
vis.Initialize()
vis.AddCamera(camera)


while vis.Run():
    
    driver.Update(vis.GetTimeStep())

    
    gator.Synchronize(vis.GetTimeStep())

    

    
    sensor_manager.Update(vis.GetTimeStep())

    
    vis.Render()

    
    my_system.DoStepDynamics(vis.GetTimeStep())