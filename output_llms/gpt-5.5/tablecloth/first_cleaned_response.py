import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


try:
    import pychrono.pardisomkl as pardisomkl
except ImportError as e:
    raise ImportError(
        "This script requires the PyChrono PardisoMKL module. "
        "Install/use a Chrono build with PardisoMKL enabled."
    ) from e






def V(x, y, z):
    
    if hasattr(chrono, "ChVector3d"):
        return chrono.ChVector3d(x, y, z)
    return chrono.ChVectorD(x, y, z)






system = chrono.ChSystemSMC()


if hasattr(system, "SetGravitationalAcceleration"):
    system.SetGravitationalAcceleration(V(0, 0, -9.81))
else:
    system.Set_G_acc(V(0, 0, -9.81))


solver = pardisomkl.ChSolverPardisoMKL()
solver.SetVerbose(False)
try:
    solver.LockSparsityPattern(True)
except Exception:
    pass
system.SetSolver(solver)


try:
    timestepper = chrono.ChTimestepperHHT(system)
    timestepper.SetAlpha(-0.2)
    timestepper.SetMaxIters(25)
    timestepper.SetAbsTolerances(1e-5)
    timestepper.SetStepControl(False)
    system.SetTimestepper(timestepper)
except Exception:
    
    pass






mesh = fea.ChMesh()
try:
    mesh.SetAutomaticGravity(True)
except Exception:
    pass


cloth_length = 0.80      
cloth_width = 0.80       
cloth_z = 0.62           

nx = 24                  
ny = 24                  

dx = cloth_length / nx
dy = cloth_width / ny


rho = 350.0              
young = 4.0e6            
poisson = 0.30           
thickness = 0.0012       



if hasattr(fea, "ChMaterialShellKirchhoffIsothropic"):
    mat = fea.ChMaterialShellKirchhoffIsothropic(rho, young, poisson)
elif hasattr(fea, "ChMaterialShellKirchhoffIsotropic"):
    mat = fea.ChMaterialShellKirchhoffIsotropic(rho, young, poisson)
else:
    raise RuntimeError(
        "Could not find ChMaterialShellKirchhoffIsothropic/Isotropic in pychrono.fea."
    )


nodes = []
for ix in range(nx + 1):
    col = []
    x = -0.5 * cloth_length + ix * dx
    for iy in range(ny + 1):
        y = -0.5 * cloth_width + iy * dy

        node = fea.ChNodeFEAxyz(V(x, y, cloth_z))

        
        if ix == 0:
            node.SetFixed(True)

        mesh.AddNode(node)
        col.append(node)

    nodes.append(col)



if not hasattr(fea, "ChElementShellBST"):
    raise RuntimeError(
        "This example requires ChElementShellBST, Chrono's Kirchhoff-Love shell element."
    )

for ix in range(nx):
    for iy in range(ny):
        n00 = nodes[ix][iy]
        n10 = nodes[ix + 1][iy]
        n01 = nodes[ix][iy + 1]
        n11 = nodes[ix + 1][iy + 1]

        
        e1 = fea.ChElementShellBST()
        e1.SetNodes(n00, n10, n11)
        e1.SetMaterial(mat)
        e1.SetThickness(thickness)
        try:
            e1.SetAlphaDamp(0.02)
        except Exception:
            pass
        mesh.AddElement(e1)

        
        e2 = fea.ChElementShellBST()
        e2.SetNodes(n00, n11, n01)
        e2.SetMaterial(mat)
        e2.SetThickness(thickness)
        try:
            e2.SetAlphaDamp(0.02)
        except Exception:
            pass
        mesh.AddElement(e2)


system.Add(mesh)







table = chrono.ChBodyEasyBox(
    1.10 * cloth_length,
    1.10 * cloth_width,
    0.04,
    1000.0,
    False,   
    True     
)
table.SetFixed(True)
table.SetPos(V(0, 0, cloth_z - 0.07))
system.Add(table)






cloth_vis = fea.ChVisualShapeFEA(mesh)
cloth_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_SURFACE)
cloth_vis.SetSmoothFaces(True)
cloth_vis.SetWireframe(False)
mesh.AddVisualShapeFEA(cloth_vis)

wire_vis = fea.ChVisualShapeFEA(mesh)
wire_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_SURFACE)
wire_vis.SetWireframe(True)
mesh.AddVisualShapeFEA(wire_vis)






vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono FEA tablecloth folding - Kirchhoff shell mesh")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(V(1.15, -1.25, 1.05), V(0.0, 0.0, cloth_z))
vis.AddTypicalLights()







pull_nodes = [nodes[nx][iy] for iy in range(ny + 1)]

total_pull_x = -2.0      
total_pull_z = 4.0       

fx_node = total_pull_x / len(pull_nodes)
fz_node = total_pull_z / len(pull_nodes)

pull_time = 1.2          
ramp_time = 0.25         

step_size = 1.0e-3

while vis.Run():
    time = system.GetChTime()

    
    if time < pull_time:
        ramp = min(time / ramp_time, 1.0)
        fx = ramp * fx_node
        fz = ramp * fz_node
    else:
        fx = 0.0
        fz = 0.0

    for node in pull_nodes:
        node.SetForce(V(fx, 0.0, fz))

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(step_size)