import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.terrain as veh_terrain
import numpy as np




time_step = 0.02  
sim_duration = 20.0
render_step = int(1 / (time_step * 50))  




sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




terrain = veh_terrain.RigidTerrain(sys)
terrain.SetContactFrictionCoefficient(0.8)
terrain.SetContactRestitutionCoefficient(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)


patch = veh_terrain.RigidTerrain.Patch(chrono.ChVectorD(0, 0, 0),
                                       chrono.ChVectorD(100, 0, 100))
patch.SetTexture(veh_terrain.Texture("concrete.jpg", 20, 20))
terrain.AddPatch(patch)




vehicle = veh.CityBus()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoorD(0, 0.5, 0))
vehicle.SetInitRotation(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.Initialize()


tire = veh.ChPacejkaTire("tire_data")
vehicle.SetTireType(tire)


sys.Add(vehicle.GetSystem())




driver = veh.ChDriver(sys)
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)
vehicle.SetDriver(driver)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("CityBus Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 2, 0), chrono.ChVectorD(0, 0, 0))
vis.SetCameraTracking(vehicle.GetChassisBody(), chrono.ChVectorD(0, 1.5, -5))


vehicle.GetChassisBody().SetVisualizationType(chrono.ChVisualization.Mesh)
vehicle.GetChassisBody().GetVisualModel().SetTexture(veh.GetDataFile("citybus/chassis_tex.jpg"))

for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        wheel.SetVisualizationType(chrono.ChVisualization.Mesh)
        wheel.GetVisualModel().SetTexture(veh.GetDataFile("citybus/wheel_tex.jpg"))




time = 0
while time < sim_duration:
    
    if vis.KeyPress(chrono.irrlicht.KEY_LEFT):
        driver.SetSteering(-0.5)
    elif vis.KeyPress(chrono.irrlicht.KEY_RIGHT):
        driver.SetSteering(0.5)
    else:
        driver.SetSteering(0)

    if vis.KeyPress(chrono.irrlicht.KEY_UP):
        driver.SetThrottle(0.8)
    elif vis.KeyPress(chrono.irrlicht.KEY_DOWN):
        driver.SetBraking(0.8)
    else:
        driver.SetThrottle(0)
        driver.SetBraking(0)

    
    sys.DoStepDynamics(time_step)
    time += time_step

    
    if vis.Run():
        vis.Render()
    else:
        break




vis.Stop()