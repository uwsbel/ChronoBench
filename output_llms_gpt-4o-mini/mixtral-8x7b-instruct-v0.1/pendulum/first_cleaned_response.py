from pychrono import *
from pychrono.irrlicht import *
from pychrono.fea import *


chrono = ChronoEngine()


vis = ChronoVisualSystemIrrlicht(chrono, "Simple Pendulum Simulation")
vis.Initialize(pyrr::vector3d(800, 600, 0))
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(pyrr::vector3d(0, 5, -15), pyrr::vector3d(0, 0, 0))


ground = chrono.AddBody(RigidBody())
ground.SetCollisionType(CollisionType.CT_RIGID)
ground_shape = BoxShape(1, 0.1, 1)
ground.AddShape(ground_shape)
ground.SetPos(pyrr::vector3d(0, -1, 0))


pendulum_length = 1.0
pendulum_mass = 1.0
pendulum_radius = 0.1
pendulum_body = chrono.AddBody(RigidBody())
pendulum_body.SetCollisionType(CollisionType.CT_RIGID)


pendulum_shape = CylinderShape(pendulum_radius, pendulum_length)
pendulum_body.AddShape(pendulum_shape)


pendulum_body.SetMass(pendulum_mass)
inertia = pendulum_shape.ComputeLocalInertia(pendulum_mass)
pendulum_body.SetInertiaXX(inertia)


pendulum_body.SetPos(pyrr::vector3d(0, 1, 0))


joint = chrono.AddJoint(RevoluteJoint)
joint.SetBody1(ground)
joint.SetBody2(pendulum_body)
joint.SetAnchor1(pyrr::vector3d(0, 0, 0))
joint.SetAnchor2(pyrr::vector3d(0, 1, 0))


chrono.SetDefaultSolver(ChSystemSolver.SR_RK4)
chrono.SetSolverType(ChSolver.Type.SR_RK4)
chrono.SetStepSize(1e-3)
chrono.SetMaxIterations(100)


log = DataLogger()
log.SetTimeInterval(0.01)
log.AddColumn("time", DataLogger.Type.REAL)
log.AddColumn("position", DataLogger.Type.VECTOR3)
log.AddColumn("velocity", DataLogger.Type.VECTOR3)


def LogData(time, system):
    pendulum_pos = pendulum_body.GetPos()
    pendulum_vel = pendulum_body.GetVel()
    log.AddRow([time, pendulum_pos, pendulum_vel])


sim_time = 5.0
while chrono.GetChTime() < sim_time:
    chrono.DoStepDynamics()
    if chrono.GetChTime() % 0.01 == 0:
        LogData(chrono.GetChTime(), chrono)

    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()


log.WriteToFile("pendulum_log.csv")


vis.Close()
chrono.Clear()