import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math






mysystem = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


myapplication = chronoirr.ChIrrApp(mysystem, 'ARTcar Simulation', chronoirr.dimension2du(800, 600))
myapplication.AddTypicalSky()
myapplication.AddTypicalLights()
myapplication.SetCameraLocation(chrono.ChVectorD(0, 2, -5))
myapplication.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))







vehicle_location = chrono.ChVectorD(0, 0.5, 0)
vehicle_orientation = chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))


vehicle = chrono.ChVehicle(mysystem, "ARTcar", vehicle_location, vehicle_orientation)


vehicle.SetContactMethod(chrono.ChContactMethodNSC())
vehicle.SetChassisVisualizationType(chrono.ChVehicle.VisualizationType_MESH)


terrain_size = chrono.ChVectorD(100, 100, 1)
terrain = chrono.ChRigidTerrain(mysystem, terrain_size, 50, 50)


terrain.SetTexture(chronoirr.GetChronoDataPath() + 'terrain/grass.png')
terrain.Initialize()






driver = chronoirr.ChIrrDriver(myapplication, vehicle)






myapplication.SetTimestep(0.01)
myapplication.SetTryRealtime(True)

while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    
    
    driver.Update(myapplication.GetDevice().GetElapsedSeconds())
    mysystem.DoStepDynamics(myapplication.GetTimestep())
    
    myapplication.EndScene()