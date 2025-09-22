import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, vehicle, irrlicht as chronoirr
from pychrono.robot import robosimian
from pychrono.vehicle import ARTcar, RigidTerrain, DrivetrainType, EngineType


sys = chrono.ChSystemNSC()


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()


car = ARTcar(sys, True, True)  


car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))  
car.SetDrivetrainType(veh.DrivetrainType_Diff4)  
car.SetEngineType(veh.EngineType_Diesel)  



wheel_visual = chrono.ChVisualShapeCylinder(0.5, 0.2)
car.AddVisualShape(wheel_visual)
wheel_visual.SetColor(chrono.ChColor(1, 0, 0))  


hood_visual = chrono.ChVisualShapeBox(1.2, 1.5, 1.0)
hood_visual.SetColor(chrono.ChColor(0, 0.5, 0))  
car.AddVisualShape(hood_visual)


engine_visual = chrono.ChVisualShapeBox(1.5, 1.0, 1.0)
engine_visual.SetColor(chrono.ChColor(0.8, 0.2, 0))  
car.AddVisualShape(engine_visual)


terrain = veh.RigidTerrain(sys)



flat_patch = terrain.AddPatch(
    patch_material='asphalt',
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=100,  
    width=50   
)


bump_patch = terrain.AddPatch(
    patch_material='grass',
    position=chrono.ChCoordsysd(chrono.ChVector3d(50, 0, 5), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=60,  
    width=40    
)


heightmap_patch = terrain.AddPatch(
    patch_material='dirt',
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=80,  
    width=60   
)



flat_visual = chrono.ChVisualShapeBox(100, 5, 5)
flat_visual.SetColor(chrono.ChColor(1, 0.5, 0))  
terrain.AddVisualShape(flat_visual)


bump_visual = chrono.ChVisualShapeMesh('bump_mesh', 1.0, 1.0)
bump_visual.SetColor(chrono.ChColor(0, 0.8, 0))  
terrain.AddVisualShape(bump_visual)


heightmap_visual = chrono.ChVisualShapeSphere(5, 5)
heightmap_visual.SetColor(chrono.ChColor(0, 0.8, 1))  
terrain.AddVisualShape(heightmap_visual)


driver = robosimian.RS_Driver(sys, 0.1)  
driver.SetSteeringDeadband(0.3)  
driver.SetThrottleRange(0.5, 0.5)  
driver.SetBrakeRange(0.5, 0.5)    


def driver_controls():
    
    speed = car.GetSpeedX()
    
    
    speed = min(speed + driver.GetThrottle() * 0.1, 12.0)  
    
    
    angle = driver.GetSteering() * 2.5  
    
    
    brake = max(0, 1 - driver.GetBrake())
    
    return speed, angle, brake


driver.RegisterDriverFunction('hmmwv_driver', driver_controls)


sys.Add(car)
sys.Add(terrain)


while True:
    
    sys.DoStepDynamics(0.01)  

    
    speed, angle, brake = driver.GetDriverFunction('hmmwv_driver')()

    
    car.SetSpeedX(speed)
    car.SetSteering(angle)
    car.SetBrake(break)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if not vis.Run():
        break