import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.postprocess as post
import numpy as np


chrono.SetChairsThreading(1)
chrono.SetChronoDataPath("chrono_data/")


system = chrono.ChSystemSMC()


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


system.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
system.SetSolver(chrono.ChSolverPardisoMKL())
system.GetSolver().SetAbsTolerances(1e-5, 1e-2)
system.GetSolver().SetVerbose(False)


vis = irr.ChIrrAppWrappers()
vis.SetCollisionVisualization(0)


tablecloth_mat = chrono.ChContactMaterialNSC()
tablecloth_mat.SetFriction(0.3)
tablecloth_mat.SetRestitution(0.2)


tablecloth_mesh = chrono.ChTriangleMeshConnected()
tablecloth_mesh.SetNumVertices(100)
tablecloth_mesh.SetNumTriangles(162)
tablecloth_mesh.ResizeNodalVector(100)
tablecloth_mesh.ResizeTriaVector(162)


nx = 10
ny = 10
dx = 0.1
dy = 0.1


for i in range(nx):
    for j in range(ny):
        tablecloth_mesh.SetVertex(i + j * nx, chrono.ChVector3d(i * dx, j * dy, 0))
for i in range(nx - 1):
    for j in range(ny - 1):
        tablecloth_mesh.AddTriangle(i, i + 1, i + 1 + nx)
        tablecloth_mesh.AddTriangle(i, i + 1 + nx, i + nx)


tablecloth_body = chrono.ChBodyEasyMesh(tablecloth_mesh, 0.001, chrono.ChContactMaterialNSC(), False, False, False)
tablecloth_body.SetPos(chrono.ChVector3d(0, 0, 1))
tablecloth_body.SetFixed(False)
tablecloth_body.SetCollide(True)
tablecloth_body.SetContactMaterial(tablecloth_mat)
system.Add(tablecloth_body)


ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.3)
ground_mat.SetRestitution(0.2)
ground_body = chrono.ChBody()
ground_body.SetFixed(True)
ground_body.SetCollide(True)
ground_body.SetContactMaterial(ground_mat)
system.Add(ground_body)


vis.SetSceneManager(irr.ChVisualSystemIrrlicht.GetClassType())
vis.AttachSystem(system)
vis.SetDefaultCameraLocation(chrono.ChVector3d(0, -0.5, 0.5))
vis.SetCollisionShapesDrawingMode(irr.VisualShapesDrawingMode_VERT_FACES)
vis.EnableCollisionVisualization(True, True)


vis.Initialize()
vis.CreateDialog("PyChrono tablecloth simulation")


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)