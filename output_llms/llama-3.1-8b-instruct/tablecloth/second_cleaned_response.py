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
        print("Error creating output directory")


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)


density = 100
E = 6e4
nu = 0.0
thickness = 0.01


melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)


L_x, L_z = 1, 1
nsections_x, nsections_z = 40, 40


mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)


def ref_X(iz, ix):
    return ix * (L_x / nsections_x)

def ref_Y(iz, ix):
    return iz * (L_z / nsections_z)


nodePlotA = mynodes[0]
nodePlotB = mynodes[nsections_x * (nsections_z + 1)]
nodesLoad = [mynodes[iz * (nsections_x + 1) + ix] for iz in range(1, nsections_z) for ix in range(1, nsections_x - 1)]


load_force = chrono.ChVector3d(0, -1, 0)


mnodemonitor = fea.ChNodeFEAxyz(chrono.ChVector3d(0.5, 0, 0))
mesh.AddNode(mnodemonitor)
melementmonitor = fea.ChElementShellBST()
mesh.AddElement(melementmonitor)


for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        if ix > 0:
            boundary_1 = mynodes[iz * (nsections_x + 1) + ix - 1]
        else:
            boundary_1 = None

        if iz > 0:
            boundary_2 = mynodes[(iz - 1) * (nsections_x + 1) + ix]
        else:
            boundary_2 = None

        if ix < nsections_x:
            boundary_3 = mynodes[iz * (nsections_x + 1) + ix + 1]
        else:
            boundary_3 = None

        if iz < nsections_z:
            boundary_4 = mynodes[(iz + 1) * (nsections_x + 1) + ix]
        else:
            boundary_4 = None

        if (iz == 0 and ix == 1):
            melementmonitor.SetNodes(mynodes[iz * (nsections_x + 1) + ix], mynodes[iz * (nsections_x + 1) + ix + 1],
                                     mynodes[(iz + 1) * (nsections_x + 1) + ix], boundary_1, boundary_2, boundary_3)
            melementmonitor.AddLayer(thickness, 0, material)
            mesh.AddElement(melementmonitor)


for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)


mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)



mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetSymbolsThickness(0.006)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mesh.AddVisualShapeFEA(mvisualizeshellB)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
vis.AddTypicalLights()


mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)


timestep = 0.005


sys.Setup()
sys.Update()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)