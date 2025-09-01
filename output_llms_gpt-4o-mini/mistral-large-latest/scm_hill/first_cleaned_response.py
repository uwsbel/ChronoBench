import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import numpy as np


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


system = chrono.ChSystemNSC()




hmmwv = veh.HMMWV_Full()
hmmwv.Initialize(system, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.6), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))))
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -3), chrono.QUNIT))
terrain.Initialize(2.0, 2.0)  


terrain.SetTexture(chrono.GetChronoDataPath() + 'textures/grass.jpg', 20, 20)
terrain.SetHeight(chrono.GetChronoDataPath() + 'terrain/height_maps/test64x64.bmp', 64, 64)


driver = veh.ChDriver(hmmwv.GetVehicle())
driver_inputs = veh.ChIrrGuiDriverInputs()
driver.Initialize(driver_inputs)




hmmwv.GetChassis().SetPos(chrono.ChVectorD(0, 0, 1.6))
hmmwv.GetChassis().SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0)))




vis = irr.ChIrrApp(system, 'HMMWV Simulation', chrono.ChCoreIrrWiz(chrono.GetChronoDataPath() + ''))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(2.5, -2.5, 10), chrono.ChVectorD(0, 0, 0), 10, 4.5, 5.5, 40, 512)
vis.EnableShadows()
vis.AddCamera(chrono.ChVectorD(0, -6, 3), chrono.ChVectorD(0, 0, 1.6))
vis.SetShowInfos(False)
vis.SetTimestep(0.01)
vis.SetTryRealtime(True)



while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()
    vis.DoStep()

    
    time = system.GetChTime()
    driver.Synchronize(time)
    driver.Advance(0.01)

    
    terrain.Synchronize(time)
    terrain.Advance(0.01)

    vis.EndScene()