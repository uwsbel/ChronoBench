import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.vehicle as vehicle


os.environ['PATH'] += ':' + os.path.dirname(chrono.__file__)
import numpy as np
import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.vehicle as vehicle


sys = chrono.ChSystemNSC()


material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()


floor = chrono.ChBody()
floor.SetPos(chrono.ChVector3d(0, -5, 0))
floor_shape = chrono.ChVisualShapeBox(10, 1, 1)
floor.AddVisualShape(floor_shape)
sys.Add(floor)

crankshaft = chrono.ChBody()
crankshaft.SetPos(chrono.ChVector3d(0, 0, 0))
crankshaft_shape = chrono.ChVisualShapeCylinder(0.5, 2)
crankshaft.AddVisualShape(crankshaft_shape)
sys.Add(crankshaft)

connecting_rod = chrono.ChBody()
connecting_rod.SetPos(chrono.ChVector3d(2, 0, 0))
connecting_rod_shape = chrono.ChVisualShapeCylinder(0.5, 2)
connecting_rod.AddVisualShape(connecting_rod_shape)
sys.Add(connecting_rod)

piston = chrono.ChBody()
piston.SetPos(chrono.ChVector3d(4, 0, 0))
piston_shape = chrono.ChVisualShapeBox(1, 1, 1)
piston.AddVisualShape(piston_shape)
sys.Add(piston)


rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(crankshaft, connecting_rod, chrono.ChFramed())
sys.Add(rev_joint)

pris_joint = chrono.ChLinkLockPrismatic()
pris_joint.Initialize(connecting_rod, piston, chrono.ChFramed())
sys.Add(pris_joint)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


sys.GetContactContainer().ReportAllContacts(chrono.ReportContactCallback())
sys.GetContactContainer().RegisterAddContactCallback(chrono.AddContactCallback())


def generate_simulation():
    sys = chrono.ChSystemNSC()
    
    return sys


def beam_simulation():
    sys = chrono.ChSystemSMC()
    mesh = fea.ChMesh()
    
    return sys

def shaking_table_simulation():
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    return sys

def robo_simian_simulation():
    sys = chrono.ChSystemSMC()
    robot = robosimian.RoboSimian(sys, True, True)
    
    return sys

def artcar_simulation():
    car = veh.ARTcar()
    
    return car