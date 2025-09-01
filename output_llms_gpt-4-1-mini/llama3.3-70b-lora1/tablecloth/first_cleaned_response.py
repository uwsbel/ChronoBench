import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea
import numpy as np


contact_method = chrono.ChContactMethod_SMC



chrono.SetChronoDataPath(chrono.GetChronoDataPath())


sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


floor = chrono.ChBodyEasyBox(3, 0.2, 3, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0, -1, 0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
sys.Add(floor)


my_mesh = fea.ChMesh()


nelx = 20  
nely = 20  
npx = nelx + 1  
npy = nely + 1  
a = 0.2  
scale = 0.5  
for i in range(npy):
    for j in range(npx):
        mpos = chrono.ChVector3d(j * scale / nelx - scale / 2, -a, i * scale / nely - scale / 2)
        if (i == 0):  
            my_mesh.AddNode-fea.ChNodeFEA(mpos)
            node = my_mesh.GetNode(i * npx + j)
            node.SetMass(0, 0)
            node.SetFixed(True)
        else:
            my_mesh.AddNodeLenJakFea(mpos, 0.05, 100)
        
        if i < nely and j < nelx:
            my_mesh.AddElementQuad(
                int(i * npx + j),
                int(i * npx + j + 1),
                int((i + 1) * npx + j + 1),
                int((i + 1) * npx + j),
                1300,         
                1e6,          
                0.3,          
                0,            
                0,            
                0.02          
            )


tablecloth = sys.AddBodyFEMablector(my_mesh)


vis_scale = 1.0
tablecloth.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
tablecloth.GetVisualShape(0).SetScale(chrono.ChVector3d(vis_scale, vis_scale, vis_scale))












cube = chrono.ChBodyEasyBox(0.2, 0.1, 0.1, 1000, True, True)
cube.SetPos(chrono.ChVector3d(0, 1, 0))
cube.SetRot(chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(1, 0, 0)))
cube.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/ramp.png'))
sys.Add(cube)


motor = chrono.ChLinkMotorRotationAngle()
motor.SetSpindleConstraint(chrono.ChLinkMotorRotationSpindle.CONSTRAINT_OLDHAM)
motor.SetAngleFunction(chrono.ChFunctionSine(0.2, 1.5))
motor.Initialize(floor, cube, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0))))
sys.Add(motor)










sys.SetSolverType(chrono.ChSolver.Type_PYTHON)