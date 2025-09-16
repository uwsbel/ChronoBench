import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import chrono.ros as ros


chrono.SetChronoDataPath("data/")
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)



terrain = chrono.ChBodyEasyBox(100, 100, 1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetBodyFixed(True)
my_system.Add(terrain)
terrain.GetMaterialSurfaceNSC().SetFriction(0.8)
terrain.GetMaterialSurfaceNSC().SetRestitution(0.2)


vehicle = veh.ChVehicle("HMMWV")
vehicle.SetChassisBody(chrono.ChBodyEasyBox(3, 1.5, 1, 1500, True, True))
vehicle.GetChassisBody().SetPos(chrono.ChVectorD(0, 1, 0))


vehicle.SetContactMethod(veh.ChContactMethod_NSC)
vehicle.SetEngineType(veh.ChVehicleEngineType_SIMPLE)
vehicle.SetTireModel(veh.ChTireModelNSC)




driver = veh.ChDriver(vehicle)


ros_manager = ros.ChROSManager(my_system)


ros_manager.RegisterClockSynchronizationHandler()
ros_manager.RegisterDriverInputHandler(driver)
ros_manager.RegisterVehicleStatePublisher(vehicle)


time_step = 0.01
while my_system.GetChTime() < 10:
    ros_manager.Update()  

    
    driver.Synchronize(time_step)
    vehicle.Synchronize(time_step)
    terrain.Synchronize(time_step)
    my_system.DoStepDynamics(time_step)

    
    ros_manager.PublishVehicleState()