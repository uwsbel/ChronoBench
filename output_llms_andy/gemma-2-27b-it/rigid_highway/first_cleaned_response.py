import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chronovehicle







my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1200, 800)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddTypicalSky()
vis.AddCamera(chrono.ChVectorD(0, 1, -5))







vehicle = chronovehicle.ChVehicleModelData(
    "HMMWV_vehicle.json"
)


vehicle_body = chronovehicle.ChVehicle(vehicle)


vehicle_body.SetPos(chrono.ChVectorD(0, 0, 0))
vehicle_body.SetRot(chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))


my_system.Add(vehicle_body)







terrain_col = chrono.ChTriangleMeshShape()
terrain_col.LoadMesh(chrono.GetChronoDataPath() + "Highway_col.obj")

terrain_vis = chrono.ChVisualShape()
terrain_vis.LoadMesh(chrono.GetChronoDataPath() + "Highway_vis.obj")


terrain = chrono.ChBody()
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.AddShape(terrain_col)
terrain.AddShape(terrain_vis)
terrain.SetBodyFixed(True)

my_system.Add(terrain)







driver = chronovehicle.ChDriver(vehicle_body)


driver.SetSteeringInput(0)
driver.SetThrottleInput(0)
driver.SetBrakingInput(0)






while vis.Run():
    
    dt = my_system.GetChTimeStep()

    
    driver.Synchronize(dt)

    
    my_system.DoStepDynamics(dt)

    
    vis.Render()