import chrono as ch
import chrono.irrlicht as chirr
import math


ch.ChEngine.Initialize()


system = ch.ChSystemNSC()


system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))


ground = ch.ChBodyEasyTerrain(
    100, 100,  20, 20, 0.1, "data/terrain/grass.jpg", "data/terrain/grass.jpg", 
    ch.ChVector3d(0, -0.1, 0), ch.ChQuaterniond(1, 0, 0, 0), True, True
)
ground.SetPos(ch.ChVector3d(0, -0.2, 0))
system.Add(ground)


turtlebot = ch.ChBody()


turtlebot.SetPos(ch.ChVector3d(0, 0.5, 0))
turtlebot.SetRot(ch.ChQuaterniond(1, 0, 0, 0))


system.Add(turtlebot)


wheel_radius = 0.1
wheel_mass = 1.0
wheel_inertia = ch.ChVector3d(0.1, 0.1, 0.1)

left_wheel = ch.ChBodyEasyCylindricalWheel(
    wheel_radius, wheel_mass, wheel_inertia, wheel_radius, ch.ChVector3d(0, 0, 0), 
    ch.ChQuaterniond(1, 0, 0, 0), False
)
left_wheel.SetPos(ch.ChVector3d(-0.2, 0.1, 0))
system.Add(left_wheel)

right_wheel = ch.ChBodyEasyCylindricalWheel(
    wheel_radius, wheel_mass, wheel_inertia, wheel_radius, ch.ChVector3d(0, 0, 0), 
    ch.ChQuaterniond(1, 0, 0, 0), False
)
right_wheel.SetPos(ch.ChVector3d(0.2, 0.1, 0))
system.Add(right_wheel)


left_joint = ch.ChLinkLockSpherical()
left_joint.Init(left_wheel, turtlebot, ch.ChFrame3d(ch.ChVector3d(-0.2, 0.1, 0), ch.ChQuaterniond(1, 0, 0, 0)))
system.Add(left_joint)

right_joint = ch.ChLinkLockSpherical()
right_joint.Init(right_wheel, turtlebot, ch.ChFrame3d(ch.ChVector3d(0.2, 0.1, 0), ch.ChQuaterniond(1, 0, 0, 0)))
system.Add(right_joint)


left_motor = ch.ChLinkMotorRotationSpeed()
left_motor.Init(left_wheel, turtlebot, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))
system.Add(left_motor)

right_motor = ch.ChLinkMotorRotationSpeed()
right_motor.Init(right_wheel, turtlebot, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))
system.Add(right_motor)


app = chirr.IrrlichtApplication(system)


app.SetCameraPosition(ch.ChVector3d(0, 5, -10))
app.SetCameraTarget(ch.ChVector3d(0, 0, 0))


app.EnableSunlight()


app.StartSimulation()


while app.GetSimulationStatus():
    
    system.Update(0.02)

    
    if app.GetCurrentTime() < 5:
        left_motor.SetSpeed(10)
        right_motor.SetSpeed(10)
    elif app.GetCurrentTime() < 10:
        left_motor.SetSpeed(-10)
        right_motor.SetSpeed(10)
    else:
        left_motor.SetSpeed(0)
        right_motor.SetSpeed(0)

    
    app.Render()


app.EndSimulation()