import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),
                         200, 200)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


driver = veh.ChIrrGuiDriver(system.GetChronoSystem())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)

driver.Initialize()


vehicle = veh.HMMWV_Full(system)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.SetVehicleStepSize(1e-3)
vehicle.Initialize()

tire_vis = veh.TireVisualizationType_PRIMITIVES
vehicle.SetTireVisualizationType(tire_vis)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Full Model Demo")
vis.Initialize()
vis.AddSkyBox()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddCamera(chrono.ChVectorD(0, 1.5, -8))
vis.AddTypicalLights()


time_step = 1e-3
render_step = 1.0 / 50
render_time = 0

while vis.Run():
    time = system.GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize("", driver_inputs)

    
    driver.Advance(time_step)
    terrain.Advance(time_step)
    vehicle.Advance(time_step)

    
    system.DoStepDynamics(time_step)

    
    if time >= render_time:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_time += render_step

    
    if time > 20:
        break


vis.Dispose()