import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.scm as scm
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  
chrono.SetChronoOutputPath("output/")


system = chrono.ChSystemSCM()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


terrain = scm.ChTerrainSCM()
terrain.SetHeightField(chrono.GetChronoDataFile("terrain/height_maps/test64.bmp"),  
                       chrono.ChVectorD(20, 0, 20),  
                       0.2)  
terrain.SetSoilParameters(2e6,  
                          0,    
                          1.1,  
                          0.0,  
                          30,   
                          0.01) 


terrain.EnableMovingPatch(True, 10.0)  
terrain.SetPlotType(scm.ChTerrainSCM::PLOT_SINKAGE, True)  


system.Add(terrain)


hmmwv = chrono.vehicle.ChWheeledVehicle(chrono.vehicle.GetDataFile("hmmwv/vehicle/HMMWV_vehicle.json"))
hmmwv.SetChassisVisualizationType(chrono.vehicle.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.vehicle.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(chrono.vehicle.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(chrono.vehicle.VisualizationType_MESH)

hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
hmmwv.SetInitFwdVel(10)  


for axle in hmmwv.GetAxles():
    for side in [chrono.vehicle.VehicleSide::LEFT, chrono.vehicle.VehicleSide::RIGHT]:
        wheel = axle.GetWheel(side)
        wheel.SetTireType(chrono.vehicle.TireType::RIGID)
        wheel.SetTireVisualizationType(chrono.vehicle.VisualizationType_MESH)


driver = chrono.vehicle.ChDriver()
hmmwv.SetDriver(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on SCM Terrain")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 3, 0), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(5, 10, 5), chrono.ChVectorD(0, 0, 0), 10, 1, 10, 40, 512)


time_step = 0.02  
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver.SetSteering(0.1 * np.sin(chrono.ChTime.GetTime()))  
    driver.SetThrottle(0.3)  
    driver.SetBraking(0.0)   

    
    system.DoStepDynamics(time_step)

    
    terrain.UpdateMovingPatch(hmmwv.GetChassis().GetPos(), hmmwv.GetChassis().GetRot())