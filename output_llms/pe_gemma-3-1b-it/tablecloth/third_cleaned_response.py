import pychrono.core as chrono  
import pychrono.irrlicht as chronoirr  
import pychrono.fea as fea  
import pychrono.pardisomkl as mkl  
import errno  
import os  


out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"  


try:
    os.mkdir(out_dir)  
except OSError as exc:
    if exc.errno != errno.EEXIST:  
        print("Error creating output directory ")


sys = chrono.ChSystemSMC()  


mesh = fea.ChMesh()  


sys.Add(mesh)



mesh.SetAutomaticGravity(False)


nodePlotA = fea.ChNodeFEAxyz()  
nodePlotB = fea.ChNodeFEAxyz()  
nodesLoad = []  


ref_X = chrono.ChFunctionInterp()
ref_Y = chrono.ChFunctionInterp()


collision_type = chrono.ChCollisionModel.Type_BULLET


collision_params = chrono.ChCollisionParams()
collision_params.SetCollisionType(collision_type)
collision_params.SetCollisionMode(chrono.ChCollisionMode_NoCollision)
collision_params.SetCollisionRadius(0.01)  


collision_setup = chrono.ChCollisionSetup()
collision_setup.SetCollisionType(collision_type)
collision_setup.SetCollisionMode(collision_params)
collision_setup.SetCollisionRadius(collision_params.SetCollisionRadius())


collision_params = chrono.ChCollisionParams()
collision_params.SetCollisionType(collision_type)
collision_params.SetCollisionMode(collision_params)
collision_params.SetCollisionRadius(collision_params.SetCollisionRadius())


collision_geometry = chrono.ChCollisionGeometry()
collision_geometry.SetCollisionShape(mesh)
collision_geometry.SetCollisionType(collision_type)
collision_geometry.SetCollisionRadius(collision_params.SetCollisionRadius())
collision_geometry.SetCollisionMode(chrono.ChCollisionMode_NoCollision)


ref_X = chrono.ChVector3d(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)), chrono.QuatFromAngleX(chrono.ChFramed(chrono.ChVector3d(0, 0, 0))))
ref_Y = chrono.ChVector3d(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)), chrono.QuatFromAngleY(chrono.ChFramed(chrono.ChVector3d(0, 0, 0))))
collision_geometry.SetReferenceFrames(ref_X, ref_Y)


load_force = chrono.ChVector3d()
load_force.SetMagnitude(1000.0) 
load_force.SetDirection(chrono.ChVector3d(0, 0, 0)) 


collision_setup.AddCollision(collision_geometry, load_force)


collision_params = chrono.ChCollisionParams()
collision_params.SetCollisionType(collision_type)
collision_params.SetCollisionMode(collision_params)
collision_params.SetCollisionRadius(collision_params.SetCollisionRadius())
collision_params.SetCollisionMode(chrono.ChCollisionMode_NoCollision)


collision_geometry = chrono.ChCollisionGeometry()
collision_geometry.SetCollisionShape(mesh)
collision_geometry.SetCollisionType(collision_type)
collision_geometry.SetCollisionRadius(collision_params.SetCollisionRadius())
collision_geometry.SetCollisionMode(chrono.ChCollisionMode_NoCollision)


collision_system = chrono.ChCollisionSystem()
collision_system.SetCollisionType(collision_type)
collision_system.SetCollisionMode(collision_params)
collision_system.SetCollisionRadius(collision_params.SetCollisionRadius())
collision_system.SetCollisionGeometry(collision_geometry)


collision_setup = chrono.ChCollisionSetup()
collision_setup.SetCollisionType(collision_type)
collision_setup.SetCollisionMode(collision_params)
collision_setup.SetCollisionRadius(collision_params.SetCollisionRadius())
collision_setup.SetCollisionGeometry(collision_geometry)


collision_params = chrono.ChCollisionParams()
collision_params.SetCollisionType(collision_type)
collision_params.SetCollisionMode(collision_params)
collision_params.SetCollisionRadius(collision_params.SetCollisionRadius())
collision_params.SetCollisionMode(chrono.ChCollisionMode_NoCollision)


collision_geometry = chrono.ChCollisionGeometry()
collision_geometry.SetCollisionShape(mesh)
collision_geometry.SetCollisionType(collision_type)
collision_geometry.SetCollisionRadius(collision_params.SetCollisionRadius())
collision_geometry.SetCollisionMode(chrono.ChCollisionMode_NoCollision)


collision_system = chrono.ChCollisionSystem()
collision_system.SetCollisionType(collision_type)
collision_system.SetCollisionMode(collision_params)
collision_system.SetCollisionRadius(collision_params.SetCollisionRadius())
collision_system.SetCollisionGeometry(collision_geometry)


collision_params = chrono.ChCollisionParams()
collision_params.SetCollisionType(collision_type)
collision_params.SetCollisionMode(collision_params)
collision_params.SetCollisionRadius(collision_params.SetCollisionRadius())
collision_params.SetCollisionMode(chrono.ChCollisionMode_NoCollision)


mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)


timestep = 0.005


sys.Setup()  
sys.Update()  


rec_X = chrono.ChFunctionInterp()
rec_Y = chrono.ChFunctionInterp()


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  

    sys.DoStepDynamics(timestep)