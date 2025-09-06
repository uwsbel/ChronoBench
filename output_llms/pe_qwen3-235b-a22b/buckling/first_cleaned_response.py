import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('path/to/chrono/data')  


sys = chrono.ChSystemSMC()


sys.SetSolverType(chrono.ChSolver.Type_MINRES)
sys.SetSolverMaxIterations(100)
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)


mesh = fea.ChMesh()


beam_L = 1.0       
beam_wy = 0.1      
beam_wz = 0.1      
young_modulus = 0.01e9  
shear_modulus = young_modulus * 0.3


msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(young_modulus)
msection.SetShearModulus(shear_modulus)
msection.SetRayleighDamping(0.0)


hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode1.SetFixed(True)  
mesh.AddNode(hnode1)

hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
mesh.AddNode(hnode2)


belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)


sys.Add(mesh)


body_end = chrono.ChBody()
body_end.SetPos(chrono.ChVector3d(beam_L, 0, 0))
body_end.SetMass(1.0)
body_end.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
sys.Add(body_end)


link_node_frame = chrono.ChLinkNodeFrame()
link_node_frame.Initialize(hnode2, body_end, chrono.ChFrameD())
sys.Add(link_node_frame)


ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)


prismatic_joint = chrono.ChLinkLockPrismatic()
prismatic_joint.Initialize(body_end, ground, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(0)))
sys.Add(prismatic_joint)


class CustomSpeedFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()
    def Get_y(self, x):
        
        return 0.1 * x  

motor = chrono.ChLinkMotorLinearSpeed()
motor.Initialize(body_end, ground, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(0)))
motor.SetSpeedFunction(CustomSpeedFunction())
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 2))
vis.AddTypicalLights()


mesh_shape = chrono.ChVisualShapeFEA(mesh)
mesh_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM)
mesh_shape.SetColor(chrono.ChColor(0, 1, 0))
mesh.AddVisualShapeFEA(mesh_shape)


body_shape = chrono.ChVisualShapeBox(0.2, 0.2, 0.2)
body_shape.SetColor(chrono.ChColor(1, 0, 0))
body_end.AddVisualShape(body_shape)


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)