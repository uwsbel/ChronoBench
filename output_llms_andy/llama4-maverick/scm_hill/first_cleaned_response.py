import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


print("Copyright (c) 2023 projectchrono.org")


init_loc = chrono.ChVectorD(0, 0, 1.0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle = veh.HMMWV_FullVeh(init_loc, init_rot)


soil_params = veh.SCMDeformableTerrain.SoilParameters()
soil_params.m_Bekker_Kphi = 0.68e6
soil_params.m_Bekker_Kc = 1.2e4
soil_params.m_Mohr_cohesion = 1.3e3
soil_params.m_Mohr_friction = 30
soil_params.m_Janosi_shear = 0.01


terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(soil_params)
terrain.Initialize(1.0, 1.0, 0.1)


terrain.InitializeHeightMap("terrain_heightmap.bmp")


driver = veh.ChDriver(vehicle)


vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0), chrono.VECT_Y)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.EnableShadows(True)
vis.AttachVehicle(vehicle)


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(driver_inputs, 0.01)
    terrain.Advance(0.01)
    driver.Advance(0.01)
    vis.Render()
    vis.Synchronize("HMMWV Simulation", driver_inputs)
    vehicle.GetSystem().DoStepDynamics(0.01)