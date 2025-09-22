import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


sys_physical = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys_physical)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('FEA ANCF cable')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 1, 0), chrono.ChVector3d(0, 1, 3))  
vis.AddTypicalLights()  


body_ground = chrono.ChBody()
sys_physical.Add(body_ground)  
body_ground.SetFixed(True)  
vis.AddBody(body_ground)  


my_mesh = fea.ChMesh()
sys_physical.Add(my_mesh)  


msection_cable = fea.ChBeamSectionCable()
msection_cable.SetDiameter(0.006)  
msection_cable.SetYoungModulus(0.01e9)  
msection_cable.SetRayleighDamping(0.000)  
msection_cable.SetSectionMassPerUnitLength(0.0)  
msection_cable.SetInertiaPerUnitLength(0.0)  
msection_cable.SetRayleighDampingPerUnitLength(0.0)  


beam_lenght = 1  


mnode1 = fea.ChNodeFEAChase5241()
mnode1.SetPos(chrono.ChVector3d(0, 0, 0))  
mnode1.SetDir(chrono.ChVector3d(1, 0, 0))  
mnode2 = fea.ChNodeFEAChase5241()
mnode2.SetPos(chrono.ChVector3d(beam_lenght, 0, 0))  
mnode2.SetDir(chrono.ChVector3d(1, 0, 0))  


my_mesh.AddNode(mnode1)
my_mesh.AddNode(mnode2)


melement = fea.ChElementBeamANCF_5241()
melement.SetNodes(mnode1, mnode2)  
melement.SetSection(msection_cable)  
my_mesh.AddElement(melement)  


mnode3 = fea.ChNodeFEAChase5241()
mnode3.SetPos(chrono.ChVector3d(beam_lenght, 0, 0))  
mnode3.SetDir(chrono.ChVector3d(1, 0, 0))  
my_mesh.AddNode(mnode3)


melement_end_force = fea.ChElementBeamANCF_5241()
melement_end_force.SetNodes(mnode2, mnode3)  
melement_end_force.SetSection(msection_cable)  
melement_end_force.ComputeGradVell();  
my_mesh.AddElement(melement_end_force)  


my_forces = chrono.ChForce()
my_forces.AddEndForce(melement_end_force, chrono.ChVector3d(0, -0.001, 0), mnode3)  
sys_physical.Add(my_forces)  


mvisualbufferbeamA = chrono.ChVisualShapeFEA()
mvisualbufferbeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZDELTA)  
mvisualbufferbeamA.SetSmoothFaces(True)  
mvisualbufferbeamA.SetWireframe(False)  
my_mesh.AddVisualShapeFEA(mvisualbufferbeamA)  


mvisualnodessmall = chrono.ChVisualShapeFEA()
mvisualnodessmall.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
mvisualnodessmall.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
mvisualnodessmall.SetSymbolsThickness(0.006)  
my_mesh.AddVisualShapeFEA(mvisualnodessmall)  


mvisualsection = chrono.ChVisualShapeFEA()
mvisualsection.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SECTION_AREA)  
mvisualsection.SetMyBeamSection(msection_cable)  
my_mesh.AddVisualShapeFEA(mvisualsection)  


hinge = chrono.ChLinkMateGeneric()
hinge.Initialize(mnode1, body_ground, False, mnode1.Frame(), mnode1.Frame())  
sys_physical.Add(hinge)  
hinge.SetConstrainedCoords(False, False, False, True, True, True)  


my_mesh.AddForce(chrono.ChForceGravity(
    sys_physical.GetGravitationalAcceleration()))  


msolver = chrono.ChSolverMINRES()
sys_physical.SetSolver(msolver)
sys_physical.GetSolver().SetMaxIterations(80)


msolver.SetDiagonalPreconditioning(True)
msolver.EnableWarmStart(True)
msolver.SetTolerance(1e-10)
msolver.SetVerbose(False)


sys_physical.SetTimestepperType(chrono.ChTimestepper.Type.EULER_IMPLICIT_LINEARIZED)


if sys_physical.GetSolver().GetType() == chrono.ChSolver.Type_MINRES:
    print("Automatically switching to the Pardiso MKL solver\n"
          " (a better sparse solver for fea problems)")
    sys_physical.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)


my_mesh.SetAutomaticGravity(False)  
mvisualbufferbeamA.SetSmoothFaces(True)  


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys_physical.DoStepDynamics(0.001)