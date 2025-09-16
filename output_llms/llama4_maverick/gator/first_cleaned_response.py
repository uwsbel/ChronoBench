import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


print('Copyright (c) 2023 Project Chrono')


gator = veh.Gator("Gator.json")


gator.SetContactMethod(chrono.ChContactMethod_SMC)


initLoc = chrono.ChVector3d(0, 0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)
gator.SetInitPosition(initLoc, initRot)


gator.SetTireType(veh.TireModelType_TMEASY)


gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(gator.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChVector3d(100, 100, 0))
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterial(terrain.GetSystem().GetContactMethod())
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.SetWindowSize(1280, 720)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(10, 10, 5), chrono.ChVector3d(0, 0, 0))
vis.AttachVehicle(gator.GetVehicle())


driver = veh.ChIrrGuiDriver(vis, gator.GetVehicle())


while vis.Run():
    time = gator.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    gator.SetDriverInputs(driver_inputs)
    gator.Update()
    terrain.Update(time)
    vis.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    vis.Render()
    vis.EndScene()
    gator.GetSystem().DoStepDynamics(0.02)