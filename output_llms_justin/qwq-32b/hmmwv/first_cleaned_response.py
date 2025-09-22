import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  


vis = irr.ChIrrApp(sys, 'HMMWV Simulation', irr.CHIRRRESIZABLE)
vis.SetCameraPosition(chrono.ChVectorD(15, 15, 15))
vis.SetCameraTarget(chrono.ChVectorD(0, 0, 0))


terrain = veh.ChRigidTerrain(sys)
patch = veh.ChTerrainPatchRectangle(
    chrono.ChVectorD(20, 20, 0.1),  
    chrono.ChVectorD(0, 0, 0),       
    chrono.ChVectorD(0, 0, 0),       
    chrono.ChMaterialSurface(),      
    'textures/grass.jpg'             
)
terrain.Initialize(patch, True)  
vis.AddAsset(patch.GetVisualShape())  


hmmwv = veh.HMMWV_Vehicle()
hmmwv.Initialize(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT),
    veh.ChTireModelType.TMEASY,      
    veh.ChVisualSystemType_PRIMITIVES  
)
sys.Add(hmmwv.GetSystem())  


driver = irr.ChIrrGuiDriver()
driver.Initialize(vis, hmmwv.GetDriverInputs())


vis.Add(hmmwv.GetVehicle())


vis.SetTimestep(0.02)  
vis.SetTryRealtime(True)


while vis.Run():
    
    driver.Synchronize(sys.GetChTime())
    
    
    hmmwv.Synchronize()
    hmmwv.Advance()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()