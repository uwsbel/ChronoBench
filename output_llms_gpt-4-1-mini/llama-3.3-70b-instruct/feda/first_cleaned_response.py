import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np


sys = chrono.ChSystemNSC()


veh_feda = veh.FEDA()
veh_feda.Initialize(sys, 
                    chrono.ChVectorD(0, 0, 1.2),  
                    chrono.ChQuaternionD(1, 0, 0, 0),  
                    0.5,  
                    veh.TireModelType.TME_SIMPLE)


sys.Add(veh_feda.GetChassisBody())
sys.Add(veh_feda.GetWheel(0))
sys.Add(veh_feda.GetWheel(1))
sys.Add(veh_feda.GetWheel(2))
sys.Add(veh_feda.GetWheel(3))


terrain = veh.RigidTerrain(sys)
terrain.SetTexture(chrono.GetChronoDataFile("asphalt.jpg"))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.AddSkyBox()
vis.AddTypicalLights()


cam = vis.AddCamera(chrono.ChVectorD(0, 0, 1.5))
cam.SetFollowNode(veh_feda.GetChassisBody())
cam.SetLookAt(chrono.ChVectorD(0, 0, 0))


for part in veh_feda.GetParts():
    vis.AssetCreateMeshVisualization(part, 
                                    chrono.ChVisualMaterial(chrono.ChVectorD(0.8, 0.8, 0.8)), 
                                    0.1)


driver = veh.InteractiveDriver(veh_feda)


time_step = 0.02
while vis.Run():
    
    sys.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    driver.Update(time_step)
    
    
    chrono.ChSleep(1/50)