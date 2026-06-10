import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math






sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))







class CustomMotorFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()

    def GetVal(self, x):
        
        
        
        A = 0.10            
        t_ramp = 2.0        
        if x < t_ramp:
            disp = A * (x / t_ramp)
        else:
            disp = A
        
        perturb = 0.0005 * math.sin(6.28 * x)
        return -(disp + perturb)   

    def Clone(self):
        return CustomMotorFunction()







mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)
sys.Add(mesh)


beam_section = fea.ChBeamSectionEulerAdvanced()


beam_width  = 0.012     
beam_height = 0.012     
beam_section.SetAsRectangularSection(beam_width, beam_height)
beam_section.SetYoungModulus(210e9)        
beam_section.SetShearModulusFromPoisson(0.3)
beam_section.SetDensity(7800.0)            
beam_section.SetRayleighDamping(0.0001)


beam_length = 1.0       
num_elements = 16       
num_nodes = num_elements + 1


beam_nodes = []
for i in range(num_nodes):
    y = beam_length * (i / num_elements)
    
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, y, 0)))
    mesh.AddNode(node)
    beam_nodes.append(node)


beam_elements = []
for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(beam_nodes[i], beam_nodes[i + 1])
    element.SetSection(beam_section)
    mesh.AddElement(element)
    beam_elements.append(element)






ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(ground)


bottom_constraint = fea.ChLinkNodeFrame()
bottom_constraint.Initialize(beam_nodes[0], ground)
sys.Add(bottom_constraint)


bottom_rot_constraint = fea.ChLinkNodeSlopeFrame()
bottom_rot_constraint.Initialize(beam_nodes[0], ground)
sys.Add(bottom_rot_constraint)


loader = chrono.ChBody()
loader.SetPos(chrono.ChVector3d(0, beam_length, 0))
loader.SetMass(1.0)
sys.Add(loader)


top_constraint = fea.ChLinkNodeFrame()
top_constraint.Initialize(beam_nodes[-1], loader)
sys.Add(top_constraint)







guide = chrono.ChBody()
guide.SetFixed(True)
guide.SetPos(chrono.ChVector3d(0, beam_length, 0))
sys.Add(guide)

motor = chrono.ChLinkMotorLinearPosition()

motor_frame = chrono.ChFramed(
    chrono.ChVector3d(0, beam_length, 0),
    chrono.QuatFromAngleX(-chrono.CH_PI_2)
)
motor.Initialize(loader, guide, motor_frame)


motor.SetMotorFunction(CustomMotorFunction())
sys.Add(motor)






vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-100.0, 100.0)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)


vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.004)
vis_nodes.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(vis_nodes)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono FEA Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, 0.6, 1.5),
              chrono.ChVector3d(0, 0.5, 0))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVector3d(2, 4, 2), 8.0)






solver = chrono.ChSolverPardisoMKL()
sys.SetSolver(solver)
solver.LockSparsityPattern(True)


timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetAlpha(-0.2)
timestepper.SetMaxiters(20)
timestepper.SetAbsTolerances(1e-6)
timestepper.SetStepControl(True)
sys.SetTimestepper(timestepper)





time_step = 0.001

while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    chronoirr.drawGrid(vis, 0.1, 0.1, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0),
                                          chrono.QuatFromAngleX(chrono.CH_PI_2)),
                       chrono.ChColor(0.4, 0.4, 0.4), True)
    vis.EndScene()

    
    sys.DoStepDynamics(time_step)

> solver = chrono.ChSolverMINRES()
> solver.SetMaxIterations(200)
> solver.SetTolerance(1e-10)
> sys.SetSolver(solver)
>