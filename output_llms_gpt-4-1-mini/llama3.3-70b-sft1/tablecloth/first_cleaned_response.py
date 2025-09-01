import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea
import pychrono.mkl as mkl
import math


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetSolverType(chrono.ChSolver.Type_PSOR)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))
ground.SetFixed(True)

system.Add(ground)


table_mat = chrono.ChContactMaterialNSC()
table = chrono.ChBodyEasyBox(2, 2, 1, 1000, True, True, table_mat)
table.SetPos(chrono.ChVector3d(0, 0, 0))
table.SetFixed(True)

system.Add(table)


msh = fea.ChMesh()


nodes = []
nx = 20
ny = 20
dx = 2.0 / (nx - 1)
dy = 2.0 / (ny - 1)
z0 = 0.4
for i in range(nx):
    x = -1 + i * dx
    for j in range(ny):
        y = -1 + j * dy
        n = fea.ChNodeFEAxyzrot(chrono.ChVector3d(x, y, z0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1)))
        n.SetMass(0.1)
        system.Add(n)
        nodes.append(n)


elems = []
for i in range(nx - 1):
    for j in range(ny - 1):
        n1 = nodes[i * ny + j]
        n2 = nodes[i * ny + j + 1]
        n3 = nodes[(i + 1) * ny + j + 1]
        n4 = nodes[(i + 1) * ny + j]
        elem = fea.ChElementShellKirchhoff(n1, n2, n3, n4, 0.1, 0.01, 1e7 / (1 - 0.3**2), 0.3, 8000)
        msh.Add(elem)
        elems.append(elem)


fix = chrono.ChLinkLockXYZROT(chrono.ChFramed(chrono.ChVector3d(-1, -1, 0.4), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1))), nodes[0])
fix.SetFixed(True)
system.Add(fix)


cube_mat = chrono.ChContactMaterialNSC()
cube_mat.SetFriction(0.5)
cube = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, cube_mat)
cube.SetPos(chrono.ChVector3d(0, -2, 1))
cube.SetRot(chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1)))
cube.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
system.Add(cube)


solver = mkl.ChSolverPardisoMKL()
system.SetSolver(solver)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Tablecloth')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2,2,2),chrono.ChVector3d(0,0.8,0))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.001)