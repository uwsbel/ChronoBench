import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())  

sys = chrono.ChSystemSMC()


solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(100)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
sys.SetSolver(solver)

sys.SetTimestepperType(chrono.ChTimestepper.Type.ALMANSS)
timestepper = sys.GetTimestepper()
timestepper.SetStepControl(True)
timestepper.SetMinStep(1e-5)
timestepper.SetMaxStep(1e-3)
timestepper.SetTolerance(1e-5)

sys.SetTimeStep(1e-4)




mesh = fea.ChMesh()


length = 1.0     
num_elements = 20
element_length = length / num_elements


E = 2.1e11        
nu = 0.3          
rho = 7800        
g = 9.81          


beam_width = 0.02  
beam_height = 0.02 
A = beam_width * beam_height
Iyy = (beam_width * beam_height ** 3) / 12.0


beam_section = fea.ChBeamSectionAdvanced()
beam_section.SetYoungModulus(E)
beam_section.SetGwithPoissonRatio(E, nu)  
beam_section.SetDensity(rho)
beam_section.SetAsRectangularSection(beam_width, beam_height)
beam_section.SetBeamRalstonShear(True)


nodes = []
for i in range(num_elements + 1):
    x = i * element_length
    node = fea.ChNodeFEAxyzrot(chrono.ChVectorD(x, 0, 0))
    mesh.AddNode(node)
    nodes.append(node)


for i in range(num_elements):
    beam_element = fea.ChElementBeamEuler()
    beam_element.SetNodes(nodes[i], nodes[i + 1])
    beam_element.SetSection(beam_section)
    beam_element.SetPitchNeutralAxis(True)
    mesh.AddElement(beam_element)


sys.Set_G_acc(chrono.ChVectorD(0, -g, 0))


sys.Add(mesh)




left_node = nodes[0]


left_node.SetFixed(True)






right_node = nodes[-1]


motor_frame = chrono.ChFrameD()


motor_frame.SetPos(right_node.GetPos())

motor = chrono.ChLinkMotorRotationAngle()
motor.SetMotorFunction(chrono.ChFunction_Ramp(-0.01, 0))  






motor_lin = chrono.ChLinkMotorLinearPosition()
motor_lin.SetName("AxialCompressionMotor")

motor_lin.Initialize(right_node, sys.GetGroundBody(), chrono.ChFrameD(right_node.GetPos()))



class AxialCompressionFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()
    def Get_y(self, x):
        
        
        
        if x < 1.0:
            return -0.01 * x
        else:
            return -0.01
    def Clone(self):
        return AxialCompressionFunction()

motor_lin.SetMotorFunction(AxialCompressionFunction())

sys.Add(motor_lin)





right_node.SetFixed(False)  




prismatic = chrono.ChLinkLockPrismaticX()
prismatic.Initialize(right_node, sys.GetGroundBody(), chrono.ChFrameD(right_node.GetPos()))
sys.Add(prismatic)








application = chronoirr.ChIrrApp(sys, "Beam Buckling Simulation with FEA and Custom Motor", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0.5, 0.3, 1.5))
application.SetTimestep(1e-4)


fea_vis = fea.ChVisualizationFEAmesh(mesh)
fea_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)  
fea_vis.SetWireframe(True)
fea_vis.SetDrawNodes(False)
mesh.AddVisualization(fea_vis)

application.AssetBindAll()
application.AssetUpdateAll()




while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()