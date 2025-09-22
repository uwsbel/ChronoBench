import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.g.VehicleJson as vehjson
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))




terrain = veh.RigidTerrain(system)


patch_length = 300.0
patch_width = 300.0
patch_height = 0.0
patch_center = chrono.ChVectorD(0, 0, patch_height)

patch = terrain.AddPatch(patch_center, chrono.QUNIT, patch_length, patch_width)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetFriction(0.9)
patch.SetRestitution(0.01)
patch.SetMaterialSurface(0, 0.9, 0.01)  

terrain.Initialize()



initLoc = chrono.ChVectorD(0, 0, 1.0)
initRot = chrono.Q_from_AngY(chrono.CH_C_PI_4)  

vehicle = veh.Kraz(vehicle_model_file=None, 
                   fixed=False, 
                   contactMethod=chrono.ChContactMethod_NSC)



vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetSystem(system)
vehicle.Initialize()





driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())


driver.SetThrottleDelta(0.05)
driver.SetSteeringDelta(0.04)
driver.SetBrakingDelta(0.1)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Kraz Vehicle Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddLogo()
vis.AttachVehicle(vehicle.GetVehicle())


cam_pos = chrono.ChVectorD(-10, -10, 5)
cam_target = vehicle.GetVehicle().GetChassisBody().GetPos()
vis.SetCameraPosition(cam_pos, cam_target)


timestep = 0.01


while vis.Run():
    
    driver_inputs = driver.GetInputs()

    
    time = system.GetChTime()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, True, False)

    
    driver.Advance(timestep)
    terrain.Advance(timestep)
    vehicle.Advance(timestep)
    vis.Advance(timestep)

    
    system.DoStepDynamics(timestep)