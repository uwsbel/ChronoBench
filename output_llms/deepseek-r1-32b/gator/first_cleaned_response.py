importchrono
importchrono.vehicle as vehicle
importchrono.visual as vis
importchrono.io as io
importchrono irrlicht as irre


system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolverType.CH_SOLVER_PGS)
system.SetSolverMaxIterations(100)
system.Set timestep(0.001)


application = irre.ChIrrApp(system, "Gator Vehicle Simulation", chronojournal.GetChronoDataPath() + " irrlicht/data")
application.SetCameraPosition(chrono.ChVectorD(0, 5, 10))
application.SetCameraRotation(chrono.ChVectorD(0, -45, 0))
application.AddTypicalLights()


terrain = vehicle.ChRigidTerrain(system)
terrain.SetDimensions(chrono.ChVectorD(100, 100, 1))
terrain.SetTexture(io.GetChronoDataFile("textures/grass.jpg"))
terrain.SetHeightMap(io.GetChronoDataFile("terrain/height_map.txt"))
application.AddBody(terrain.GetBody())


gator = vehicle.ChGator()
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)))
gator.SetContactMethod(chrono.ChContactMethod.NODE)
gator.SetTireType(vehicle.ChTireType.TMEASY)
gator.SetUse_mesh(True)


system.Add(gator.GetChassis())
for wheel in gator.GetWheelSet().GetWheelList():
    system.Add(wheel.GetBody())
    application.AddBody(wheel.GetBody())


driver = vehicle.ChDriver(gator.GetDriver())
driver.SetThrottle(0)
driver.SetSteering(0)
driver.SetBraking(0)


def key_press(key, mod):
    if key == 'a':
        driver.SetSteering(-0.5)
    elif key == 'd':
        driver.SetSteering(0.5)
    elif key == 'w':
        driver.SetThrottle(0.5)
    elif key == 's':
        driver.SetBraking(0.5)

application.keyboard().RegisterCallback(key_press)


while application.GetDevice().Run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    
    
    driver.Update()
    
    
    system.DoStepDynamics(0.001)
    
    
    application.GetDevice().Sleep(1/50)

print("Simulation stopped")