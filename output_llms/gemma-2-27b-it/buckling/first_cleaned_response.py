import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


beam_material = fea.ChMaterialShellANCF(rho=7850, E=200e9, nu=0.3)
beam_section = fea.ChSectionShellParametric(
    radius=0.05, thickness=0.01, material=beam_material
)
beam_length = 1.0
beam = fea.ChBeamSectionCable(beam_section, beam_length)


beam_body = chrono.ChBody()
beam_body.SetBodyFixed(False)
beam_body.SetPos(chrono.ChVectorD(0, 0, 0))
my_system.Add(beam_body)


mesh = fea.ChMesh()
nodes = []
for i in range(10):
    nodes.append(fea.ChNodeFEAxyz(chrono.ChVectorD(i * beam_length / 9, 0, 0)))
mesh.AddNodes(nodes)
elements = []
for i in range(9):
    elements.append(fea.ChElementBeamANCF(nodes[i], nodes[i + 1], beam))
mesh.AddElements(elements)


fea.ChMeshExporter.ExportToChrono(mesh, beam_body)


def apply_load(time):
    return 1000 * time  


force_element = chrono.ChForce()
force_element.SetMode(chrono.ChForce.FORCE)
force_element.SetDir(chrono.ChVectorD(0, 1, 0))
force_element.SetVpoint(chrono.ChVectorD(beam_length, 0, 0))
force_element.SetMforce(apply_load)
beam_body.AddForce(force_element)


fixed_constraint = chrono.ChLinkLockRevolute()
fixed_constraint.Initialize(beam_body, chrono.ChBody(), chrono.ChVectorD(0, 0, 0))
my_system.Add(fixed_constraint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Beam Buckling Simulation")
vis.Initialize()


my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
my_system.SetStep(0.001)


while vis.Run():
    my_system.DoStepDynamics(0.001)
    vis.Render()