import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemSMC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


length = 10  
num_elements = 10  
density = 100  
young_modulus = 2e6  
diameter = 0.1  


beam = chrono.ChCableANCF(length, num_elements, density, young_modulus, diameter)
sys.Add(beam)


constraint = chrono.ChLinkMateFix()
constraint.Initialize(ground, beam.GetNode(0))
sys.Add(constraint)


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetTitle('ANCF Cable Beam Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()