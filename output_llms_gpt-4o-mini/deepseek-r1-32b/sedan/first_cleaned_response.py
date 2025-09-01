importchrono
importchrono.vehicle as vehicle
importchrono.vehicle.tire as tire
importchrono.sensor as sensor
importchrono irrlicht as vis


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = vehicle.BMW_E90()
vehicle.SetChassisMass(1500)
vehicle.SetChassisDimensions(length=4.5, width=1.8, height=1.4)
vehicle.SetPosition(chrono.ChVectorD(0, 0, 1))


terrain = chrono.ChPlane()
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.SetNormal(chrono.ChVectorD(0, 1, 0))
terrain.SetTexture("textures/gravel.jpg", 10, 10)
system.Add(terrain)


tire_params = tire.TMeasyParams()
tire_params.radius = 0.35
tire_params.width = 0.25
tire_params.pressure = 2.5e6
for wheel in vehicle.GetWheels():
    tm_tire = tire.TMeasyTire(tire_params)
    wheel.SetTire(tm_tire)


driver = vehicle.Driver()
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)


visualizer = vis.ChIrrApp(system, "BMW E90 Simulation", 1024, 768)
visualizer.AddLight(chrono.ChVectorD(0, 10, 0), chrono.ChVectorD(0.8, 0.8, 0.8))
visualizer.AddSkybox("skybox/sky.xml")
visualizer.AddChaseCamera(vehicle.GetChassis(), 5, 1)


terrain_texture = chrono.ChTexture()
terrain_texture.SetTextureFilename("textures/terrain.jpg")
terrain_texture.SetRepeatX(10)
terrain_texture.SetRepeatY(10)
terrain.GetMaterial().AddTexture(terrain_texture)


vehicle.SetChassisVisualizationType(vehicle.ChassisVisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(vehicle.WheelVisualizationType_MESH)


def key_callback(key, pressed):
    if key == 'left':
        driver.SetSteering(-0.5)
    elif key == 'right':
        driver.SetSteering(0.5)
    elif key == 'up':
        driver.SetThrottle(0.5)
    elif key == 'down':
        driver.SetBraking(0.5)
    elif key == 'space':
        driver.Reset()

visualizer.SetKeyCallback(key_callback)


visualizer.Run()
while visualizer.Run():
    driver.Update()
    vehicle.Update()
    time = system.GetChTime()
    print(f"Simulation time: {time:.2f}s")


visualizer.Close()