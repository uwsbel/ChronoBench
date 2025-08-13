import os
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(os.path.join(os.getcwd(), 'data'))  
sys = chrono.ChSystemSMC()  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(chrono.ChMaterialSurfaceSMC(), 
                          chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                          100.0, 100.0)  
patch.SetContactMaterial(chrono.ChMaterialSurfaceSMC())
patch.SetMeshFilename('Highway_col.obj')  
patch.SetVisualMeshFilename('Highway_vis.obj')  
terrain.Initialize()


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod.SMC)
hmmwv.SetChassisCollisionType(veh.HMMWV.ChassisCollisionType.PRISMATIC)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngZ(0)))  
hmmwv.Initialize()


for tire in hmmwv.GetTires():
    tire.SetTireType(veh.HMMWV.TireModelType.TMEASY)  


driver = veh.HMMWV_Driver(hmmwv)
driver.SetThrottle(0.5)  
driver.SetSteering(0.0)  
driver.SetBraking(0.0)   


time_step = 1 / 50.0  
while vis.Run():
    
    driver.UpdateInputs()  

    
    sys.DoStepDynamics(time_step)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


vis.Close()