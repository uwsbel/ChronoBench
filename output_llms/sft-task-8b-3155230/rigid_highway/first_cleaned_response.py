import pychrono as chrono
from pychrono.vehicle import ChSystem, ChSystemSMC, ChTireType_TMEASY, ChDriver, ChHMMWV, ChHMMWV_Full, ChVisualSystemIrrlicht


system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on Custom Mesh Terrain')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 3.5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


vehicle = ChHMMWV_Full()
vehicle.SetName('HMMWV')
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChVector3d(0, 0.5, 1.0))
vehicle.SetTireType(ChTireType_TMEASY)
vehicle.SetTireStepSize(0.01)
vehicle.SetEngineType(chrono.ChVehicleEngineType.ENGINE_EMS)
vehicle.SetDriveType(chrono.ChVehicleDriveType.FWD)
vehicle.SetBrakeType(chrono.ChVehicleBrakeType.SHORTEST)
vehicle.SetSteeringType(chrono.ChVehicleSteeringType.FULL)
vehicle.SetTireFrictionCoefficient(1.0)
vehicle.SetTireLongitudinalStiffness(100000)
vehicle.SetTireLongitudinalDamping(1000)
vehicle.SetTireLateralStiffness(100000)
vehicle.SetTireLateralDamping(1000)
vehicle.SetTireRollingResistanceCoefficient(0.01)
vehicle.SetTireContactMethod(chrono.ChTireContactMethod.CONTACT_SMC)
vehicle.SetTireVisualizationType(chrono.ChTireVisualizationType.VISUALIZATION_MESH)
vehicle.SetTireMeshFile('tire.obj')
vehicle.SetTireMeshScale(0.01)
vehicle.SetTireMeshColor(chrono.ChColor(0.5, 0.5, 0.5))


system.Add(vehicle)


terrain_collision = chrono.ChVisualShapeMesh()
terrain_collision.SetFilename('Highway_col.obj')
terrain_collision.SetColor(chrono.ChColor(0.1, 0.1, 0.1))
terrain_collision.SetOpacity(0.5)
terrain_visual = chrono.ChVisualShapeMesh()
terrain_visual.SetFilename('Highway_vis.obj')
terrain_visual.SetColor(chrono.ChColor(0.8, 0.8, 0.8))


terrain_body = chrono.ChBody()
terrain_body.SetName('Custom Terrain')
terrain_body.SetFixed(True)
terrain_body.AddVisualShape(terrain_collision)
terrain_body.AddVisualShape(terrain_visual)
system.Add(terrain_body)


driver = ChDriver(vehicle)
driver.SetSteeringInput(0.0)
driver.SetThrottleInput(0.0)
driver.SetBrakingInput(0.0)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    
    if vis.GetKeyboard(chrono.KEYBOARD_KEY_LEFT):
        driver.SetSteeringInput(-0.1)
    elif vis.GetKeyboard(chrono.KEYBOARD_KEY_RIGHT):
        driver.SetSteeringInput(0.1)
    else:
        driver.SetSteeringInput(0.0)
    
    if vis.GetKeyboard(chrono.KEYBOARD_KEY_UP):
        driver.SetThrottleInput(0.1)
    elif vis.GetKeyboard(chrono.KEYBOARD_KEY_DOWN):
        driver.SetThrottleInput(-0.1)
    else:
        driver.SetThrottleInput(0.0)
    
    if vis.GetKeyboard(chrono.KEYBOARD_KEY_SPACE):
        driver.SetBrakingInput(0.1)
    else:
        driver.SetBrakingInput(0.0)
    
    
    driver.Synchronize(vehicle.GetChTime())
    driver.Advance(vehicle.GetChTimeStep())
    
    vis.EndScene()
    system.DoStepDynamics(0.001)