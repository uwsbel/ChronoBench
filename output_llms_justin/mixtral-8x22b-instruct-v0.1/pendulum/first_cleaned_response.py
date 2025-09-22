import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data")  
chrono.SetChronoDataPath("/path/to/irrlicht/data")  


physics_system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(width=10, height=0.1, length=10, density=1000)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
physics_system.Add(ground)


pendulum_mass = 1
pendulum_radius = 0.1
pendulum_inertia = 0.5 * pendulum_mass * pendulum_radius**2
pendulum = chrono.ChBodyEasySphere(radius=pendulum_radius, density=pendulum_mass)
pendulum.SetPos(chrono.ChVectorD(0, 1, 0))
pendulum.SetInertiaXX(chrono.ChVectorD(pendulum_inertia, pendulum_inertia, pendulum_inertia))
physics_system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
physics_system.Add(joint)


app = chronoirr.ChIrrApp(physics_system, "PyChrono Pendulum Simulation", chronoirr.dimension2du(800, 600), True, True)
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(0, 1, -1.5))
app.AddLogo(chrono.GetChronoDataFile("logo_pychrono.png"))
app.AddShadowAll()


ground_shape = chronoirr.ChIrrCylinderShape(ground, 5, 0.1, True, True)
ground_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
pendulum_shape = chronoirr.ChIrrShape(pendulum, chronoirr.ChIrrCylinderShape(pendulum, pendulum_radius, 0.01, True, True))
pendulum_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
joint_shape = chronoirr.ChIrrLineShape(joint, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), 0.01)
joint_shape.SetColor(chrono.ChColor(0.2, 0.8, 0.2))


log_file = open("pendulum_log.txt", "w")
log_file.write("Time,Position,Velocity\n")


while app.GetDevice().run():
    physics_system.DoStepDynamics(0.01)
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()

    
    pendulum_position = pendulum.GetPos()
    pendulum_velocity = pendulum.GetPos_dt()
    log_file.write(f"{physics_system.GetChTime():.2f},{pendulum_position.y:.4f},{pendulum_velocity.y:.4f}\n")

log_file.close()