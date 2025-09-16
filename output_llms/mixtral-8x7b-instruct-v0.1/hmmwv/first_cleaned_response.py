import chrono as chrono
import irrlicht


chrono.ChronoPy_SetFlags(chrono.ChSystem.Visualization | chrono.ChSystem.CSV_LOGGER)
my_system = chrono.ChSystem()



my_terrain = chrono.ChTerrain()
my_system.Add(my_terrain)
my_terrain.SetBox(-100, 100, -100, 100, 0, 0, chrono.ChMaterialSurfaceBase(), 10, 10, 0.5)
my_terrain.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'), 10, 10)


vehicle_location = chrono.ChVectorD(0, 0, 0)
vehicle_orientation = chrono.Q_from_Euler(chrono.ChVectorD(0, 0, 0))
chassis = chrono.ChBodyEasyBox(0.5, 0.2, 2.5, 1000, chrono.ChVectorD(0, 0, 0), vehicle_orientation)
my_system.Add(chassis)
chassis.SetPos(vehicle_location)
chassis.SetCollide(True)


tire_radius = 0.4
tire_width = 0.2
tire_spacing = 1.3
tire_positions = [chrono.ChVectorD(-tire_spacing, 0.2, -tire_radius),
                   chrono.ChVectorD(-tire_spacing, 0.2, tire_radius),
                   chrono.ChVectorD(tire_spacing, 0.2, -tire_radius),
                   chrono.ChVectorD(tire_spacing, 0.2, tire_radius)]
for i in range(4):
    tire = chrono.ChBodyEasyCylinder(tire_width, tire_radius, True, chrono.ChVectorD(0, 0, 0), chrono.Q_from_Euler(chrono.ChVectorD(0, 0, 0)))
    tire.SetMass(50)
    tire.SetPos(tire_positions[i])
    tire.SetCollide(True)
    my_system.Add(tire)
    tire_contact = chrono.ChTmeasyTire(tire, chassis, tire_radius, tire_width)
    my_system.Add(tire_contact)


driver = chrono.ChInteractiveDriver()
my_system.Add(driver)
driver.SetChassis(chassis)
driver.SetSteerScalar(0.1)
driver.SetThrottleScalar(0.1)
driver.SetBrakeScalar(0.1)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.SetChronoSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_white_small.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.BeginScene()
vis.DrawAll()
vis.EndScene()


t0 = chrono.ChTime()
while(vis.Run()):
    step = chrono.ChTime(0.02)  
    my_system.DoStepDynamics(step)
    my_system.DoStepCollision(step, my_terrain)
    vis.DrawAll()
    vis.Synchronize()
    dt = chrono.ChTime() - t0
    t0 = chrono.ChTime()
    if dt < step:
        chrono.ChSleep(step - dt)