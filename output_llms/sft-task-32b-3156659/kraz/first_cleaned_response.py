import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as chronoirr
import time


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


kraz = vehicle.ChVehicleSystemKraz(sys, "Kraz_Vehicle")
init_pos = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
kraz.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
kraz.Initialize()


terrain = vehicle.ChRigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.AddPatch(patch_mat, chrono.ChCoordsysd(), 100.0, 100.0)
terrain.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Kraz Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 1.5), chrono.ChVector3d(0, 0, 0))
vis.AddLightDirectional()
vis.EnableShadows(True)

driver = vehicle.ChIrrGuiDriver(vis)
driver.SetSteeringClamp(1.0)
driver.SetThrottleClamp(1.0)
driver.SetBrakingClamp(1.0)
driver.SetSteeringSensitivity(4)
driver.SetThrottleSensitivity(4)
driver.SetBrakingSensitivity(4)


time_step = 0.001
simulation_time = 0.0
end_time = 100.0

while vis.Run():
    
    simulation_time = sys.GetChTime()
    driver.Synchronize(simulation_time)
    kraz.Synchronize(simulation_time, driver.GetInputs(), terrain)
    sys.DoStepDynamics(time_step)
    
    
    vehicle_pos = kraz.GetVehiclePos()
    vis.GetActiveCamera().SetPos(chrono.ChVector3d(vehicle_pos.x - 5, vehicle_pos.y, vehicle_pos.z + 2))
    vis.GetActiveCamera().LookAt(vehicle_pos)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    time.sleep(max(0.0, time_step - (time.time() - simulation_time)))