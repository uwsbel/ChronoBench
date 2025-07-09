import pychrono
import pychrono as chrono
import numpy as np
import math


chrono.SetChronoDataPath('/path/to/chrono/data')


sys = chrono.ChSystemNSC()


beam_L = 10.0  
beam_wy = 0.1  
beam_wz = 0.1  
density = 7850  


hnode1 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))


msection = chrono.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(2.1e11)  
msection.SetShearModulus(0.8e9)  
msection.SetRayleighDamping(0.000)  
belement1 = chrono.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)


sys.Add(hnode1)
sys.Add(hnode2)
sys.Add(belement1)


g = chrono.ChVector3d(0, -9.81, 0)


hnode1.SetPos(chrono.ChVector3d(0, 0, 0))
hnode2.SetPos(chrono.ChVector3d(beam_L, 0, 0))


hnode1_shape = chrono.ChVisualShapeBox(beam_wy, beam_wz, 0.1)
hnode2_shape = chrono.ChVisualShapeBox(beam_wy, beam_wz, 0.1)
hnode1.AddVisualShape(hnode1_shape)
hnode2.AddVisualShape(hnode2_shape)


belement1_shape = chrono.ChVisualShapeBox(beam_wy, beam_wz, 0.1)
belement1.AddVisualShape(belement1_shape)


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Simulation')
vis.Initialize()


time_step = 0.001
time_end = 10.0
num_steps = int(time_end / time_step)

for i in range(num_steps):
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


vis.Run()