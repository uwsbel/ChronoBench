import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea
import numpy as np
import math as m


system = chrono.ChSystemNSC()


system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))
ground.SetFixed(True)
system.Add(ground)


beam = fea.ChBeam3d()


Ia_x = 0.0005
Ia_y = 0.0005
Jzz = 0.0005
A = 0.01
KGA = 1e7
as_section = fea.RigidSectionA(Ia_x, Ia_y, Jzz, A, KGA)
beam.SetSection(as_section)


n = 10
L = 10
mesh = fea.ChMeshBeam3d()
for i in range(n):
    nA = mesh.AddNode(chrono.ChVector3d(L * i / (n - 1), 0, 0))
    if i == 0:
        mesh.SetConstraint(nA, fea.DirectoryDir_X | fea.DirectoryDir_Y | fea.DirectoryDir_Z | fea.DirectoryRot_X | fea.DirectoryRot_Y | fea.DirectoryRot_Z)
    elif i == n - 1:
        mesh.SetConstraint(nA, fea.DirectoryDir_X | fea.DirectoryDir_Y | fea.DirectoryDir_Z)
    else:
        mesh.SetConstraint(nA, 0)
    if i < n - 1:
        nB = mesh.AddNode(chrono.ChVector3d(L * (i + 1) / (n - 1), 0, 0))
        mesh.AddElement(beam, nA, nB)
mesh.Initialize()


beam_FEA = fea.ChBeam3dFEA(mesh)
system.Add(beam_FEA)


cube = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, ground_mat)
cube.SetPos(chrono.ChVector3d(5, 0, 0))
cube.SetFixed(False)
system.Add(cube)


spring = chrono.ChLinkSpring()
spring.SetMounts(chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChFramed(cube.GetPos(), chrono.QUNIT))
spring.SetSpringLength(0)
spring.SetStiffness(1e5)
spring.SetDamping(1e3)
system.Add(spring)


cube2 = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, ground_mat)
cube2.SetPos(chrono.ChVector3d(10, 0, 0))
cube2.SetFixed(False)
system.Add(cube2)


spring2 = chrono.ChLinkSpring()
spring2.SetMounts(chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChFramed(cube2.GetPos(), chrono.QUNIT))
spring2.SetSpringLength(0)
spring2.SetStiffness(1e5)
spring2.SetDamping(1e3)
system.Add(spring2)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test beam FEA')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, 2, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


time_step = 1e-4
time_end = 2

while vis.Run() :
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)