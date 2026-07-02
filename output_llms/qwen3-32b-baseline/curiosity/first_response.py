import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr
import math

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# 2. Add fixed ground with collision properties
ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
ground.GetCollisionModel().SetCollisionType(chrono.ChCollisionModelBase.ChCollisionType.STATIC)
system.Add(ground)

# Add texture to ground
tex = chrono.ChTexture()
tex.SetTextureFilename(chrono.GetChronoDataPath() + 'concrete.jpg')
ground.AddAsset(tex)

# 3. Create Curiosity rover model
rover = vehicle.Mars_rover("CuriosityRover")
rover.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
rover.SetSuspensionVisualizationType(vehicle.VisualizationType_MESH)
rover.SetWheelVisualizationType(vehicle.VisualizationType_MESH)
rover.SetSteeringVisualizationType(vehicle.VisualizationType_MESH)
rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngY(0)))
system.Add(rover)

# Add collision shapes to wheels (example for one wheel - extend as needed)
wheel = rover.GetWheel(0, vehicle.VehicleSide_LEFT)
wheel.GetSpindle().GetBody().GetCollisionModel().AddBox(0.3, 0.3, 0.3, chrono.ChVectorD(0, 0, 0))

# 4. Set up motor control driver
driver = vehicle.ChDriver(rover)
driver.SetSteering(0.5)  # Initial steering
driver.SetThrottle(1.0)  # Full throttle

# 5. Initialize Irrlicht visualization
application = irr.ChIrrApp(system, 'Curiosity Rover Simulation', irr.dimension2du(1024, 768))
application.SetSkyBox()
application.SetCamera(chrono.ChVectorD(0, 3, -8), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
application.SetLightDirection(chrono.ChVectorD(1, -1, -1))
application.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono.png')
application.AddTypingText()
application.AddFPS()
application.SetShadows()

# 6. Simulation loop
time_step = 0.001
time = 0

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    # Update driver with time-based steering (sinusoidal for demonstration)
    steering_angle = 0.5 * math.sin(time)
    driver.SetSteering(steering_angle)
    driver.Update(time_step)
    
    system.DoStepDynamics(time_step)
    time += time_step
    
    application.EndScene()