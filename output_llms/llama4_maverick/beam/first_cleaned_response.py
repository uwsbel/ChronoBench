import pychrono as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()


mesh = chrono.ChMesh()


mesh.SetAutomaticGravity(True)


beam_mat = chrono.ChMaterialBeamANCF(0.01, 100, 0.01)


num_nodes = 10
for i in range(num_nodes):
    node = chrono.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(i, 0, 0)))
    mesh.AddNode(node)


for i in range(num_nodes - 1):
    beam = chrono.ChElementBeamEuler(
        mesh.GetNode(i),
        mesh.GetNode(i + 1),
        0.1,
        beam_mat,
        10,
        chrono.ChVector3d(0, 1, 0)
    )
    mesh.AddElement(beam)


sys.Add(mesh)


vis_mesh = chrono.ChVisualShapeFEA(mesh)
vis_mesh.SetWireframe(True)
mesh.AddVisualShapeFEA(vis_mesh)


app = chronoirr.ChIrrApp(sys, "Beam FEA Simulation", chronoirr.dimension2du(800, 600))


while app.GetDevice().run():
    sys.DoStepDynamics(0.01)
    app.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.EndScene()