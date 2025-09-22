import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.engine as eng
import pychrono.tire as tire
import numpy as np


chrono.SetChronoDataPath('./chrono_data/')
system = chrono.ChSystemNSC()


veh_hmmwv = veh.HMMWV()
veh_hmmwv.SetContactMethod(chrono.ChContactMethod_MULTICORE_SDL)
veh_hmmwv.SetChassisFixed(False)
veh_hmmwv.SetEngineType(eng.ChEngineType_INTERNALCOMBUSTION)
veh_hmmwv.SetTireModel(tire.ChTireModel_RIGID_RING)


terrain = chrono.ChTerrain()
terrain.SetFriction(0.8)
terrain.SetRestitution(0.2)
terrain.SetSize(100, 100, 100)


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(veh_hmmwv)
driver.SetSteering(0.0)
driver.SetThrottle(0.5)
driver.SetBraking(0.0)


ros_manager = chrono.ChROSManager()
ros_manager.RegisterHandler(chrono.ChROSHandler_ClockSync())
ros_manager.RegisterHandler(chrono.ChROSHandler_DriverInputs(driver))
ros_manager.RegisterHandler(chrono.ChROSHandler_VehicleState(veh_hmmwv))


system.SetStep(0.01)
system.SetMaxItersSolverSpeed(100)
system.SetMaxItersSolverStab(100)


while True:
    
    system.Synchronize(chrono.ChSystemNSC.SYNC_POS)
    system.Synchronize(chrono.ChSystemNSC.SYNC_VEL)
    
    
    system.DoStepDynamics(0.01)
    terrain.DoStep(0.01)
    driver.DoStep(0.01)
    
    
    ros_manager.PublishData()
    
    
    if system.GetChTime() > 10.0:
        break