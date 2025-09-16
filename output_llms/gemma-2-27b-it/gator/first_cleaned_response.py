import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math






mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
mysystem.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
mysystem.SetSolverMaxIterations(100)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(mysystem)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Gator Vehicle Simulation")
vis.Initialize()


ground_material = chrono.ChMaterialSurfaceNSC()
ground_material.SetFriction(0.8)
ground_material.SetRestitution(0.2)

ground = chrono.ChBodyEasyBox(
    20, 20, 1, 1000, True, True, ground_material
)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
mysystem.Add(ground)


gator = chrono.ChVehicle(mysystem)


gator.SetChassisFixed(False)
gator.SetChassisCollision(True)
gator.SetChassisMaterial(ground_material)
gator.SetChassisBody(chrono.ChBodyEasyBox(2, 1, 0.5, 100, True, True))
gator.GetChassisBody().SetPos(chrono.ChVectorD(0, 1, 0))


wheel_radius = 0.3
wheel_width = 0.15
wheel_mass = 10
wheel_inertia = chrono.ChVectorD(0.5, 0.5, 0.5)


wheel_positions = [
    chrono.ChVectorD(0.8, 0.5, 0.5),
    chrono.ChVectorD(-0.8, 0.5, 0.5),
    chrono.ChVectorD(0.8, 0.5, -0.5),
    chrono.ChVectorD(-0.8, 0.5, -0.5),
]


for i in range(4):
    wheel = chrono.ChWheel(
        wheel_radius,
        wheel_width,
        wheel_mass,
        wheel_inertia,
        chrono.ChVectorD(0, 0, 1),
        chrono.ChVectorD(0, -1, 0),
    )
    wheel.SetVisualizationType(chrono.ChWheel.VisualizationType_MESH)
    wheel.SetContactMethod(chrono.ChWheel.ContactMethod_NSC)
    wheel.SetTireModel(chrono.ChTireModelTMeasy())
    gator.AddWheel(wheel)
    gator.SetWheelPosition(wheel, wheel_positions[i])


gator.GetChassisBody().SetRot(chrono.Q_from_AngAxis(math.pi / 2, chrono.ChVectorD(0, 1, 0)))






class Driver:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.steering = 0
        self.throttle = 0
        self.brake = 0

    def ApplyControls(self):
        self.vehicle.SetSteeringValue(self.steering)
        self.vehicle.SetThrottle(self.throttle)
        self.vehicle.SetBraking(self.brake)

driver = Driver(gator)






while vis.Run():
    
    keys = chronoirr.GetKeyboard()
    if keys.GetPressedKey(chrono.irrlicht.KEY_LEFT):
        driver.steering = -1
    elif keys.GetPressedKey(chrono.irrlicht.KEY_RIGHT):
        driver.steering = 1
    else:
        driver.steering = 0

    if keys.GetPressedKey(chrono.irrlicht.KEY_UP):
        driver.throttle = 1
    elif keys.GetPressedKey(chrono.irrlicht.KEY_DOWN):
        driver.brake = 1
    else:
        driver.throttle = 0
        driver.brake = 0

    
    driver.ApplyControls()

    
    mysystem.DoStepDynamics(0.02)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    chrono.ChTime.Wait(1.0 / 50.0)