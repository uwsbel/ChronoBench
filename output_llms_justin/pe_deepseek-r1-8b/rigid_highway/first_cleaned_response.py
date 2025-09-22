import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot
from pychrono.terrain import veh
from pychrono.robots import HMMWV
from pychrono.irrlicht import chronoirr
from pychrono.vehicle import vehicle


sys = chrono.ChSystemNSC()
chrono.SetChronoDataPath("data")  


hmmvw = HMMWV(sys)
hmmvw.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))  
hmmvw.SetContactMethod("hinge")  
hmmvw.SetTireModel("TMEASY")  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.AddLogo(chrono.GetChronoDataFile("pychrono.png"))
vis.AddSkyBox()
cam_pos = chrono.ChVector3d(5, 5, 10)
vis.AddCamera(cam_pos)
vis.Initialize()


terrain = veh.RigidTerrain(sys)

collision_mesh = terrain.AddCollisionMesh("Highway_col.obj")
terrain.Initialize(collision_mesh)

visual_mesh = terrain.AddVisualMesh("Highway_vis.obj")
terrain.GetVisuals().SetMaterial(visual_mesh, chrono.ChColor(1, 0.5, 0.5))  


terrain_link = chrono.ChLinkLockPrismatic()
terrain_link.Initialize(terrain, hmmvw, chrono.ChCoordsysd(chrono.ChVector3d(0, -5, 0), chrono.QuatFromAngleX(chrono.CH_PI/2)))
sys.Add(terrain_link)


driver = vehicle.RSDriver(sys, "driver")
driver.SetSteeringFunction(chrono.ChFunction_Sine(0.1, 1.0))  
driver.SetThrottleFunction(chrono.ChFunction_Sine(0.1, 1.0))  
driver.SetBrakeFunction(chrono.ChFunction_Sine(0.1, 1.0))    
driver.EnableDriver(True)


sys.Add(hmmvw)


fps = 50  
time_step = 1.0 / fps  


def OnReportContact(self, pA, pB, plane_coord, distance, eff_radius, cforce, ctorque, modA, modB):
    print(f"Contact detected between body A and B at point {pA}")


sys.GetContactContainer().RegisterAllContactCallbacks(OnReportContact)


while True:
    
    driver.Update()
    
    
    sys.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if vis.WindowCloseRequested():
        break


vis.Terminate()