import pychrono as chrono
from chrono.core import ChVectorD, ChQuaternionD, ChFrameD, ChFunction_Const
import pychrono.fem as fem
import pychrono.irrlicht as irr




system = chrono.ChSystemSMC()


system.Set_G_acc(ChVectorD(0.0, 0.0, 0.0))


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(200)


dt = 1.0e-5
system.SetStepSize(dt)





L_beam = 1.0                
n_control_points = 11       
dx = L_beam / (n_control_points - 1)


iga_mesh = fem.ChIsogeometricMesh()


kv = fem.ChKnotVector()
kv.SetKnots([0, 0, 0, 0,
             0.25, 0.5, 0.75,
             1, 1, 1, 1])
iga_mesh.SetKnotVector(kv)


nodes = []
for i in range(n_control_points):
    pos = ChVectorD(i * dx, 0.0, 0.0)
    node = fem.ChNodeFEMbody(ChFrameD(pos, ChQuaternionD(1, 0, 0, 0)))  
    
    node.SetMass(0.01)
    iga_mesh.AddControlPoint(node)   
    nodes.append(node)


beam_elem = fem.ChElementBeamIGA()
beam_elem.SetMesh(iga_mesh)


section = fem.ChBeamSectionCircular()
section.SetDiameter(0.05)            
section.SetYoungModulus(2.1e11)     
section.SetShearModulus(8.0e10)
section.SetDensity(7850.0)           
beam_elem.SetSection(section)


system.Add(iga_mesh)




flywheel = chrono.ChBody()
flywheel.SetMass(5.0)                              
flywheel.SetInertiaXX(ChVectorD(0.04, 0.04, 0.02))  
flywheel.SetPos(ChVectorD(L_beam / 2.0, 0.0, 0.0))


sph = chrono.ChSphereShape(0.15)
flywheel.AddShape(sph, ChFrameD())
system.AddBody(flywheel)


mid_node = nodes[len(nodes) // 2]          
flywheel_joint = chrono.ChLinkLock()
flywheel_joint.Initialize(flywheel, mid_node,
                          ChFrameD(), ChFrameD())   
system.AddLink(flywheel_joint)





ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(ChVectorD(0.0, 0.0, 0.0))
system.AddBody(ground)


motor_joint = chrono.ChLinkRevolute()
motor_joint.Initialize(ground, nodes[0],
                       ChFrameD(ChVectorD(0.0, 0.0, 0.0), ChQuaternionD(1, 0, 0, 0)))
system.AddLink(motor_joint)


motor_joint.SetMotorEnable(True)
motor_joint.SetMotorSpeed(ChFunction_Const(100.0))   




vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor – IGA Beam")
vis.AddLight(irr.ChVectorf(0, 10, 0), 5.0)
vis.AddCamera(irr.ChVectorf(-2, -1, -2), irr.ChVectorf(0.5, 0, 0))
vis.Initialize()




time = 0.0
end_time = 2.0                     

print("Starting simulation …")
while vis.Run():
    if time >= end_time:
        break

    system.DoStepDynamics(dt)
    time += dt

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

print(f"Simulation finished at t = {time:.3f} s")