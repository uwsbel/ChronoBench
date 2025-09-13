import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath("./chrono_data/")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



vehicle = veh.ChVehicleMAN10T(system)


vehicle.SetPos(chrono.ChVectorD(0, 1, 0))
vehicle.SetTransform(chrono.ChFrameD(chrono.ChVectorD(0, 1, 0), chrono.ChQuaternionD(1, 0, 0, 0)))


vehicle.SetCollidable(True)
vehicle.SetVisualize(True)
vehicle.SetVehicleDebugCollision(False)  



terrain = chrono.ChBodyEasy()
terrain.SetBodyFixed(True)
terrain.SetMass(0)
terrain.SetPos(chrono.ChVectorD(0, -0.1, 0))
terrain.SetShapeBoxes(0.2, 0.05, 2)  
terrain.CreateMaterialAndTexture(chrono.ChMaterialSurfaceNSC(), "terrain.png")
system.Add(terrain)



vehicle.SetTireFormula(veh.ChTireTMEasy())



class MyDriver:
    def __init__(self):
        self.throttle = 0.0
        self.steering = 0.0
        self.braking = 0.0

    def Drive(self, vehicle, time_step):
        
        if chrono.ChTime() < 5:
            self.throttle = 0.5
            self.steering = 0.0
            self.braking = 0.0
        elif chrono.ChTime() >= 5 and chrono.ChTime() < 10:
            self.throttle = 0.5
            self.steering = 0.2
            self.braking = 0.0
        else:
            self.throttle = 0.0
            self.steering = 0.0
            self.braking = 0.5

        vehicle.SetThrottle(self.throttle)
        vehicle.SetSteering(self.steering)
        vehicle.SetBraking(self.braking)

driver = MyDriver()



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 1, 0))
vis.AddTypicalLights()
vis.AddSkyBox()


vis.GetSceneManager().addTexture("terrain.png", "terrain.png")
vis.GetSceneManager().addLogo("chrono_logo.png")


vis.SetChaseCameraMode()


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    driver.Drive(vehicle, time_step)
    system.DoStepDynamics(time_step)