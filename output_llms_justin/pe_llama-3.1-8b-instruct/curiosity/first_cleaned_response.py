import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as irrlicht
import pychrono.vehicle as vehicle


os.chdir(os.path.dirname(__file__))
chrono.SetChronoDataPath('./chrono_data')
np.set_printoptions(precision=4, suppress=True)


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetDefaultSuggestedEnvelope(0.01)
sys.SetDefaultSuggestedMargin(0.005)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3d(0, -1, 0))
ground_shape = chrono.ChVisualShapeBox(10, 1, 10)
ground.AddVisualShape(ground_shape)
ground.EnableCollision(True)
sys.Add(ground)

rover = chrono.ChBody()
rover.SetPos(chrono.ChVector3d(0, 1, 0))
rover_shape = chrono.ChVisualShapeBox(2, 1, 2)
rover.AddVisualShape(rover_shape)
rover.EnableCollision(True)
sys.Add(rover)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, rover, chrono.ChFramed())
sys.Add(joint)


t = 0
dt = 0.01
while t < 10:
    sys.DoStepDynamics(dt)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    t += dt


def report_contact(pA, pB, plane_coord, distance, eff_radius, cforce, ctorque, modA, modB):
    print(f'Contact detected between {pA} and {pB}')

sys.GetContactContainer().ReportAllContacts(report_contact)