import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.terrain as terrain


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


hmmwv = veh.HMMWV_Full(system)
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetEngineType(veh.EngineType_SHAFTS)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0)))
hmmwv.Initialize()


terrain = terrain.RigidTerrain(system)
mat = chrono.ChMaterialSurfaceNSC()
mat.SetFriction(0.9)
mat.SetRestitution(0.01)
terrain.SetMaterial(mat)
terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.png"))
terrain.Initialize()


hmmwv.Initialize(terrain)


driver = veh.ChDriver(hmmwv.GetVehicle())


ros_manager = chros.ChronoROSManager()
ros_manager.RegisterClockHandler()
ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(driver, "~/driver_inputs"))
ros_manager.RegisterHandler(chros.ChROSBodyStateHandler(hmmwv.GetVehicle(), "~/vehicle/state"))
ros_manager.Initialize()


time_step = 0.01
sim_time = 0.0
total_sim_time = 10.0


while sim_time < total_sim_time:
    
    driver.Synchronize(sim_time)
    hmmwv.Synchronize(sim_time, driver.GetInputs(), terrain)
    
    
    system.DoStepDynamics(time_step)
    sim_time += time_step

    
    ros_manager.Update(sim_time, time_step)