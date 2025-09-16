import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.WheeledVehicle(system, chrono.GetChronoDataFile("vehicle/bmw_e90/bmw_e90.json"))
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0), chrono.QUNIT))


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


vehicle.GetChassis().SetCollide(True)
for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        wheel.SetCollide(True)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)
ground.GetCollisionModel().AddBox(chrono.ChVectorD(100, 1, 100))
ground.SetPos(chrono.ChVectorD(0, -1, 0))
system.Add(ground)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(100, 1, 100)
ground.GetAssets().push_back(ground_shape)
ground_texture = chrono.ChTexture()
ground_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.GetAssets().push_back(ground_texture)


tires = []
for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        tire = veh.TMeasyTire("TMeasy")
        tire.Initialize(wheel, veh.Wheel.SIDE_NONE)
        tires.append(tire)


app = irr.ChIrrApp(system, 'BMW E90 Simulation', irr.dimension2du(1024, 768))
app.AddSkyBox()
app.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 1, 0))
app.AddLightWithShadow(chrono.ChVectorD(10, 20, -10), chrono.ChVectorD(0, 0, 0), 90, 0.2, 100, 0.5, chrono.ChColor(1,1,1))
app.AssetBindAll()
app.AssetUpdateAll()


driver = veh.ChInteractiveDriverIRR(app.GetDevice())
driver.Initialize()


chase_camera = veh.ChChaseCamera(vehicle.GetChassisBody())
chase_camera.SetDistance(6.0)
chase_camera.SetHeight(1.5)
chase_camera.SetPitch(-chrono.CH_C_DEG_TO_RAD * 15)
chase_camera.SetChasePosition(chrono.ChVectorD(0, 1.2, 0))


while app.GetDevice().run():
    time = system.GetChTime()

    
    driver.Synchronize(time)
    driver_inputs = driver.GetInputs()

    
    vehicle.Synchronize(time, driver_inputs)

    
    system.DoStepDynamics(1e-3)

    
    chase_camera.Update()
    irr_cam = app.GetSceneManager().getActiveCamera()
    pos = chase_camera.GetPosition()
    target = chase_camera.GetTarget()
    irr_cam.setPosition(irr.core.vector3df(pos.x, pos.y, pos.z))
    irr_cam.setTarget(irr.core.vector3df(target.x, target.y, target.z))

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()