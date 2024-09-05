import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chronovehicle







system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
system.SetSolverType(chrono.ChSolver.Type_PSOR)
system.SetSolverMaxIterations(100)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("CityBus Simulation")
vis.Initialize()


camera = vis.GetSceneManager().addCameraSceneNode()
camera.setPosition(chrono.ChVectorD(0, 5, -15))
camera.setTarget(chrono.ChVectorD(0, 0, 0))







vehicle = chronovehicle.ChVehicle("CityBus")
vehicle.SetChassisFixed(False)
vehicle.SetChassisBody(chrono.ChBodyEasyBox(2.5, 1.0, 5.0, 1000))
vehicle.GetChassisBody().SetPos(chrono.ChVectorD(0, 1.0, 0))


num_wheels = 4
wheel_radius = 0.4
wheel_width = 0.2
for i in range(num_wheels):
    wheel = chronovehicle.ChWheel(chrono.ChBodyEasyBox(wheel_width, wheel_radius, wheel_width, 100))
    wheel.SetPos(chrono.ChVectorD(
        (-1.5 + i * 1.5, -0.5, 1.5) if i < 2 else (-1.5 + (i - 2) * 1.5, -0.5, -1.5)
    ))
    vehicle.AddWheel(wheel)


for wheel in vehicle.GetWheels():
    wheel.SetTireModel(chronovehicle.ChTireModelNSC())







terrain = chrono.ChRigidTerrain()
terrain.SetTexture(chrono.GetChronoDataPath() + "terrain/textures/grass.png")
terrain.Initialize(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(100, 100, 100))
system.Add(terrain)







class Driver:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.steering = 0
        self.throttle = 0
        self.brake = 0

    def Update(self, dt):
        
        
        if vis.GetDevice().IsKeyDown(chrono.irr::KEY_LEFT):
            self.steering = -1
        elif vis.GetDevice().IsKeyDown(chrono.irr::KEY_RIGHT):
            self.steering = 1
        else:
            self.steering = 0

        if vis.GetDevice().IsKeyDown(chrono.irr::KEY_UP):
            self.throttle = 1
        elif vis.GetDevice().IsKeyDown(chrono.irr::KEY_DOWN):
            self.brake = 1
        else:
            self.throttle = 0
            self.brake = 0

        self.vehicle.SetSteering(self.steering)
        self.vehicle.SetThrottle(self.throttle)
        self.vehicle.SetBraking(self.brake)


driver = Driver(vehicle)







system.Add(vehicle)


dt = 1.0 / 50.0

while vis.Run():
    
    driver.Update(dt)

    
    system.DoStepDynamics(dt)

    
    camera.setPosition(vehicle.GetChassisBody().GetPos() + chrono.ChVectorD(0, 5, -15))
    camera.setTarget(vehicle.GetChassisBody().GetPos())

    
    vis.Render()