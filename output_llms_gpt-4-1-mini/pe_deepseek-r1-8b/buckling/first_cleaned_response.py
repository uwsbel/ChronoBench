import os
import math
import numpy as np
import pychrono as chrono
from pychrono import fea
from pychrono.robots import robosimian
from pychrono.vehicles import veh
from pychrono.irrlicht import chronoirr


sys = chrono.ChSystemNSC()


chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/data/'


beam_section = fea.ChBeamSectionEulerAdvanced()
beam_section.SetAsRectangularSection(0.1, 0.05, 0.01)  
beam_section.SetYoungModulus(210e9)  
beam_section.SetShearModulus(210e9 * 0.3)  
beam_section.SetRayleighDamping(0.000)  


support1 = chrono.ChBody()
support1.SetPos(chrono.ChVector3d(0, 0, 0))
support1.EnableCollision(False)
support1.AddVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(1, 0.5, 0.05)))
sys.Add(support1)

support2 = chrono.ChBody()
support2.SetPos(chrono.ChVector3d(4, 0, 0))
support2.EnableCollision(False)
support2.AddVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(1, 0.5, 0.05)))
sys.Add(support2)


moving_mass = fea.ChMovingMass()
moving_mass.SetMass(0.01)  
moving_mass.SetSize(0.1, 0.1, 0.01)  
moving_mass.SetVisualShape(chrono.ChVisualShapeSphere(0.05))  
moving_mass.SetInitialPosition(chrono.ChVector3d(1, 0, 0))
moving_mass.SetCollision(False)
sys.Add(moving_mass)


timestepper = fea.ChTimeStepper()
timestepper.SetTimeStep(0.001)
timestepper.Initialize(sys)
timestepper.SetStartTime(0)
timestepper.SetStopTime(5)
timestepper.Start()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddSkyBox()
cam_pos = chrono.ChVector3d(2, 2, 2)
vis.AddCamera(cam_pos)


beam_node1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
beam_node2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(4, 0, 0)))
beam_element = fea.ChElementBeamEuler()
beam_element.SetNodes(beam_node1, beam_node2)
beam_element.SetSection(beam_section)
fea.AddElement(beam_element)
sys.Add(beam_element)


mfunX = chrono.ChFunction_Sine(0.1, 0.5, 0)  
mfunY = chrono.ChFunction_Const(0.01)  
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(moving_mass, support2, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
motor.SetSpeedFunction(chrono.ChFunction_Sine(100, 0.5))


sys.Add(motor)


vis.AddLogo(os.path.join(chrono.GetChronoDataFile('pychrono'), 'pychrono.png'))
vis.AddTypicalLights()


while vis.Run():
    timestepper.DoStepDynamics()
    sys.DoStepDynamics(timestepper.GetTimeStep())
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    num_contacts = sys.GetNumContacts()
    if num_contacts > 0:
        print(f"Number of contacts: {num_contacts}")