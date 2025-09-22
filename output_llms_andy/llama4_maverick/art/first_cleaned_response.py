import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


print("Copyright (c) 2023 projectchrono.org")


init_loc = chrono.ChVectorD(0, 0.5, -1.0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
contact_method = chrono.ChContactMethod_SMC
vehicle = veh.ARTcarVehicle(contact_method)


vehicle.SetContactMethod(contact_method)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()


vis_type = veh.VisualizationType_PRIMITIVES
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), chrono.ChVectorD(10, 10, 1))
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterialProperties(2e7, 0.3)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 10, 10)
terrain.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowTitle('ARTcar Demo')
vis.SetWindowSize(1280, 720)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 3, 1), init_loc)
vis.AttachVehicle(vehicle)


driver = veh.ChIrrGuiDriver(vehicle.GetVehicle(), vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(time, driver_inputs)
    terrain.Advance(time)
    vis.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    vis.Render()
    irr.drawAllCOGs(vis, chrono.ChVectorD(0, 0, 0))
    vis.EndScene()
    vis.Synchronize("ARTcar Simulation", driver.GetInputModeAsString())
    chrono.ChRealtimeStep(vehicle.GetSystem(), 1 / 50)