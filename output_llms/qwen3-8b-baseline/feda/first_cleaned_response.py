import pychrono as chrono
from pychrono import irrlicht
import pychrono.irrlicht as irr
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))
system.SetTimestep(1.0 / 50)


application = irr.ChIrrApp(system, "FEDA Vehicle Simulation", irr.dimension2d_u32(800, 600))
application.AddCamera(chrono.ChCamera())
application.SetCameraPosition(chrono.ChVectorD(0, 2, 5))
application.SetCameraTarget(chrono.ChVectorD(0, 0, 0))


terrain = chrono.ChBodyEasyBox(100, 0.1, 100, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.SetFixed(True)
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.8)
material.SetRestitution(0.2)
terrain.GetMaterialSurface().SetMaterial(0, material)
terrain.GetMaterialSurface().SetTexture("grass.png")  
system.Add(terrain)



chassis = chrono.ChBodyEasyBox(2, 1, 0.5, 1000, True, True)
chassis.SetPos(chrono.ChVectorD(0, 0.25, 0))
chassis.SetFixed(False)
chassis.SetMaterialSurface(material)
system.Add(chassis)


wheel_radius = 0.3
wheel_mass = 20
wheel_inertia = 0.5 * wheel_mass * wheel_radius**2


wheel_fl = chrono.ChBodyEasyCylinder(wheel_radius, 0.5, wheel_mass, True, True)
wheel_fl.SetPos(chrono.ChVectorD(-1, 0.25, 0))
wheel_fl.SetFixed(False)
wheel_fl.SetMaterialSurface(material)
system.Add(wheel_fl)


wheel_fr = chrono.ChBodyEasyCylinder(wheel_radius, 0.5, wheel_mass, True, True)
wheel_fr.SetPos(chrono.ChVectorD(1, 0.25, 0))
wheel_fr.SetFixed(False)
wheel_fr.SetMaterialSurface(material)
system.Add(wheel_fr)


wheel_rl = chrono.ChBodyEasyCylinder(wheel_radius, 0.5, wheel_mass, True, True)
wheel_rl.SetPos(chrono.ChVectorD(-1, 0.25, -2))
wheel_rl.SetFixed(False)
wheel_rl.SetMaterialSurface(material)
system.Add(wheel_rl)


wheel_rr = chrono.ChBodyEasyCylinder(wheel_radius, 0.5, wheel_mass, True, True)
wheel_rr.SetPos(chrono.ChVectorD(1, 0.25, -2))
wheel_rr.SetFixed(False)
wheel_rr.SetMaterialSurface(material)
system.Add(wheel_rr)


for wheel in [wheel_fl, wheel_fr, wheel_rl, wheel_rr]:
    spring = chrono.ChSpring()
    spring.SetSpringStiffness(10000)
    spring.SetDamping(100)
    spring.SetRestLength(0.5)
    spring.Connect(chassis, wheel)
    system.Add(spring)


def update_driver():
    
    if application.GetDevice().isKeyDown(irr.KEY_KEY_A):
        chassis.Rotate(chrono.ChVectorD(0, 0, 1), 0.05)
    elif application.GetDevice().isKeyDown(irr.KEY_KEY_D):
        chassis.Rotate(chrono.ChVectorD(0, 0, 1), -0.05)
    
    
    if application.GetDevice().isKeyDown(irr.KEY_KEY_W):
        
        force = chrono.ChVectorD(0, 0, 1000)
        chassis.ApplyForce(force, chassis.GetPos(), True)
    if application.GetDevice().isKeyDown(irr.KEY_KEY_S):
        
        force = chrono.ChVectorD(0, 0, -1000)
        chassis.ApplyForce(force, chassis.GetPos(), True)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    update_driver()
    
    
    camera_pos = chassis.GetPos() + chrono.ChVectorD(0, 2, 5)
    application.SetCameraPosition(camera_pos)
    application.SetCameraTarget(chassis.GetPos())
    
    application.EndScene()