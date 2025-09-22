import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  
sys = chrono.ChSystemNSC()  
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


beam_length = 2.0  
beam_width = 0.1   
beam_height = 0.1  
num_elements = 10  


beam = chrono.ChBodyEasyMesh(
    filename='',  
    density=7800,  
    make_collide=True,
    make_visual=True
)


beam_shape = chrono.ChBoxShape()
beam_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(beam_length, beam_height, beam_width))
beam.AddAsset(beam_shape)


sys.Add(beam)


nodes = []
elements = []


for i in range(num_elements + 1):
    x = -beam_length/2 + i * beam_length/num_elements
    node = chrono.ChNodeFEAxyzrot(x, 0, 0)
    nodes.append(node)
    sys.Add(node)


for i in range(num_elements):
    element = chrono.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetYoungModulus(2.1e11)  
    element.SetBeamInertiaY(beam_height*beam_width**3/12)
    element.SetBeamInertiaZ(beam_width*beam_height**3/12)
    element.SetBeamSection(beam_height*beam_width)
    elements.append(element)
    sys.Add(element)


class CustomMotor(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        self.time = 0
        self.force_magnitude = 1000  

    def Get_y(self, time):
        self.time = time
        
        if time < 1:
            return self.force_magnitude * time
        else:
            return self.force_magnitude


motor_force = CustomMotor()
force = chrono.ChForce()
force.SetMode(chrono.ChForce.FORCE)
force.SetF_y(motor_force)
nodes[-1].AddForce(force)


constraint = chrono.ChLinkLockRevolute()
constraint.Initialize(nodes[0], chrono.ChFrameD(chrono.ChVectorD(-beam_length/2, 0, 0)))
sys.Add(constraint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, -5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 5.5, -2.5), chrono.ChVectorD(0, 0, 0), 3, 4, 10, 40, 512)


sys.SetSolverType(chrono.ChSolver.Type_MINRES)
sys.SetMaxIterationSolverSpeed(100)
sys.SetMaxIterationSolverStab(100)
sys.SetTolForce(1e-10)

timestepper = chrono.ChTimestepperHHT(sys)
timestepper.SetAlpha(-0.2)
timestepper.SetMaxIters(10)
timestepper.SetTol(1e-10)
sys.SetTimestepper(timestepper)


time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)
    vis.Advance(time_step)

    
    current_force = motor_force.Get_y(vis.GetChTime())
    current_displacement = nodes[-1].GetPos().y
    print(f"Time: {vis.GetChTime():.2f}s, Force: {current_force:.1f}N, Displacement: {current_displacement:.4f}m")