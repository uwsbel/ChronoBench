from pychrono.core import *
from pychrono.vehicle_robot import *
from pychrono.vehicle_robot import VehicleRobot, VehicleRobotDriver
from pychrono import visualization
from pychrono.irrlicht_utils import *


chrono_system = ChronoSystem()
chrono_system.SetGravity(0, 0, -9.81)


ground = ChronoFixedBody()
ground.SetPosition(0, 0, 0)
chrono_system.AddSolidBody(ground)


viper_rover = VehicleRobot()
viper_driver = VehicleRobotDriver(viper_rover)


viper_rover.SetPosition(0, 0, 0)
viper_rover.SetOrientation(0, 0, 0, 1)


chrono_system.Add(viper_rover)


irrlicht_application = visualization.IrrlichtApplication(chrono_system, "Viper Rover Simulation")
irrlicht_application.SetWindowSize(800, 600)
irrlicht_application.SetWindowCaption("Viper Rover Simulation")


camera = irrlicht_application.AddCamera()
camera.SetPosition(10, 10, 10)
camera.SetFocalPoint(0, 0, 0)
camera.SetFieldOfView(45)


light = irrlicht_application.AddLight()
light.SetPosition(10, 10, 10)
light.SetColor(255, 255, 255)


irrlicht_application.AddLight(light)


renderer = irrlicht_application.AddRenderWindow()


irrlicht_application.AddCameraSceneNode(camera)
irrlicht_application.AddLightSceneNode(light)


while irrlicht_application.GetDevice()->run():
    
    chrono_system.DoStep(0.001, 1)
    
    
    irrlicht_application.SetGamma(270)
    irrlicht_application.drawScene()
    irrlicht_application.DoFrame()
    
    
    viper_driver.SetSteeringAngle(viper_driver.GetSteeringAngle() + 0.01)  
    
    
    viper_rover.SetPosition(viper_rover.GetPosition())
    viper_rover.SetOrientation(viper_rover.GetOrientation())
    irrlicht_application.drawScene()
    irrlicht_application.DoFrame()


irrlicht_application.MainLoop()