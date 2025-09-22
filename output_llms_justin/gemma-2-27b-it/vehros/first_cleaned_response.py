import pychrono as chrono
import pychrono.vehicle as chrono_vehicle
import pychrono.irrlicht as chrono_irr
import roslibpy
import time


chrono.SetChronoDataPath("data/")
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




terrain = chrono_vehicle.ChTerrainPlane()
terrain.SetFriction(0.8)
terrain.SetRestitution(0.2)
my_system.Add(terrain)


vehicle = chrono_vehicle.ChVehicle("HMMWV")
vehicle.SetContactMethod(chrono_vehicle.ChContactMethod_NSC)
vehicle.SetEngineType(chrono_vehicle.ChVehicleEngineType_SIMPLE)
vehicle.SetTireModel(chrono_vehicle.ChTireModelType_RIGID)


vehicle.Initialize(my_system)




vehicle.SetPos(chrono.ChVectorD(0, 0.5, 0))


driver = chrono_vehicle.ChDriver(vehicle)




ros_node = roslibpy.Ros(host='localhost', port=9090)






while True:

    
    

    
    

    
    my_system.DoStepDynamics(0.01)

    
    

    
    

    time.sleep(0.01)