import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


veh.InitChrono()


vehicle = veh.CityBus()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 1.0), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetInitFwdVel(5.0)
vehicle.SetTireType(veh.TireType_TMeasy)
vehicle.Initialize()


terrain = veh.RigidTerrain(vehicle.GetSystem())
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
terrain.SetContactMaterial(chrono.ChMaterialSurface.SMC)
terrain.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/terrain.obj"))
mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
terrain.AddMesh(mesh, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('CityBus Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.SetDriverInputs(driver_inputs)
    vehicle.Advance(0.02)
    vis.Advance(0.02)
    vis.Render()