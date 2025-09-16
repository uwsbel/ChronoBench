import pychrono as p
import pychrono.irrlicht as irr
import pychrono.physics as phys
import numpy as np


p.init()


win = irr.IrrlichtApplication.createWindow(
    "Simple Pendulum",
    1280,
    720,
    irr.irr_display_type.e_display_window
)


ground = p.createBody(p.BodyType.e_ground)
ground.position = p.Vector3(0, 0, 0)
ground.inertia = p.Inertia(mass=100, Ixx=100, Iyy=100, Izz=100)


pendulum = p.createBody(p.BodyType.e_rigidbody)
pendulum.mass = 1
pendulum.inertia = p.Inertia(mass=1, Ixx=0.1, Iyy=0.1, Izz=0.1)
pendulum.position = p.Vector3(0, 0.5, 0)
pendulum.rotation = p.Quaternion(0, 0, 0, 1)


joint = p.createRevoluteJoint(
    pendulum, ground, p.Vector3(0, 0, 0), p.Vector3(0, 0, 1)
)


dt = 0.01
time = 0.0


p.setDeltaTime(dt)
p.setSimTime(time)


while True:
    
    p.update()

    
    pendulum_position = pendulum.position
    pendulum_velocity = pendulum.velocity

    
    print("Pendulum Position:", pendulum_position)
    print("Pendulum Velocity:", pendulum_velocity)

    
    win.render()

    
    time += dt