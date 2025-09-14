import os
import math
import time
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




CONTACT_METHOD = chrono.ChContactMethod_SMC
STEP_SIZE = 0.005
VISUALIZATION_FPS = 50  




system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))




initial_pos = chrono.ChVectorD(0, 0, 0.5)
initial_rot = chrono.Q_ROTATE_Y_TO_Z  

artcar = veh.ARTVehicle(
    system,
    initial_pos,
    initial_rot,
    contact_method=CONTACT_METHOD,
    visualization_type=veh.VisualizationType_MESH
)
artcar.Initialize()




terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(0, 0, 1),
    100.0,  
    100.0   
)
patch.SetTexture(veh.GetChronoDataFile("terrain/textures/concrete.jpg"), 100, 100)
terrain.Initialize()




driver = veh.InteractiveDriver(artcar.GetVehicle())
artcar.GetVehicle().SetDriver(driver)




vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle("ARTcar Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1.5))
vis.AddTypicalLights()


driver.SetInputDataFile(veh.GetDataFile("driver/InputData.txt"))
driver.Initialize()
vis.AddUserEventReceiver(driver.GetInputReceiver())




frame_interval = 1.0 / VISUALIZATION_FPS
last_frame_time = time.time()

while vis.Run():
    
    current_time = time.time()
    if (current_time - last_frame_time) < frame_interval:
        time.sleep(frame_interval - (current_time - last_frame_time))
    last_frame_time = current_time

    
    time = system.GetChTime()
    driver.Synchronize(time)
    artcar.Synchronize(time)
    system.DoStepDynamics(STEP_SIZE)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    irr.ChIrrTools.drawAllStats(
        vis,
        artcar.GetVehicle().GetSteeringController().GetControlModeString(),
        True
    )
    irr.ChIrrTools.drawGUI(vis)


vis.GetDevice().closeDevice()