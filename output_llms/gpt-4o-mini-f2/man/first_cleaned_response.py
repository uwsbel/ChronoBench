import pychrono as chrono
import pychrono.robot as robot
import math


system = chrono.ChSystemNSC()


robot = robot.QuadRotor(system)
robot.SetInitPosition(chrono.ChVector3d(0, 0, 10))
robot.SetFixed(False)
robot.SetChassisCollisionType(chrono.ChCollisionObject.CollisionType_NONE)
robot.SetChassisVisualType(chrono.ChVisualShape.VisualType_MESH)
robot.SetRotorVisualType(chrono.ChVisualShape.VisualType_MESH)
robot.SetRotorCollisionType(chrono.ChCollisionObject.CollisionType_NONE)
robot.SetBladeVisualType(chrono.ChVisualShape.VisualType_MESH)
robot.SetBladeCollisionType(chrono.ChCollisionObject.CollisionType_NONE)
robot.SetCamera(chrono.ChVector3d(0, 0, 0.5), chrono.ChVector3d(0, 0, 0))
robot.SetTargetFramerate(60)


robot.Initialize()


ground = chrono.ChBodyEasyBox(100, 100, 1, 1000, True, True)
ground.SetPos(chrono.ChVector3d(0, 0, -1))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


force = chrono.ChForceGeneratorPoint()
force.SetBody(ground)
force.SetForce(chrono.ChVector3d(0, 0, 10))
force.SetApplicationPoint(chrono.ChVector3d(0, 0, 0))
system.Add(force)


time_keeper = robot.ChTimeKeeper()
time_keeper.SetTime(0)


while time_keeper.GetTime() < 10:
    time_keeper.Step()
    robot.Update()
    system.DoStepDynamics(1e-3)