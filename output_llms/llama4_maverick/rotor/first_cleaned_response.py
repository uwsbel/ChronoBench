import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("./chrono_data/")  


sys = chrono.ChSystemSMC()



beam_length = 1.0
beam_radius = 0.05
num_elements = 10
beam_mat = fea.ChMaterialBeamEuler(chrono.ChVectorD(210e9, 0.3, 0.0),  
                                   chrono.ChVectorD(1, 1, 1))  

beam = fea.ChElementBeamEuler(beam_mat, num_elements, beam_length / num_elements, beam_radius)


for i in range(num_elements + 1):
    node_pos = chrono.ChVectorD(i * beam_length / num_elements, 0, 0)
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(node_pos))
    sys.Add(node)
    if i == 0:
        
        node.SetFixed(True)


for i in range(num_elements):
    node_start = sys.Get_otherphysicsitem(fea.ChNodeFEAxyzrot, i)
    node_end = sys.Get_otherphysicsitem(fea.ChNodeFEAxyzrot, i + 1)
    beam = fea.ChElementBeamEuler(beam_mat)
    beam.SetNodes(node_start, node_end)
    sys.Add(beam)


flywheel_body = chrono.ChBodyEasyCylinder(chrono.ChVectorD(beam_length / 2, 0, 0),  
                                          0.2,  
                                          0.1,  
                                          1000,  
                                          True)  
sys.Add(flywheel_body)


center_node = sys.Get_otherphysicsitem(fea.ChNodeFEAxyzrot, num_elements // 2)
link = chrono.ChLinkMateGeneric(True, True, True, True, True, True)
link.Initialize(flywheel_body, center_node, False, center_node.GetFrame_REF_to_abs(), center_node.GetFrame_REF_to_abs())
sys.Add(link)


motor = chrono.ChLinkMotorRotationSpeed(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
motor.Initialize(sys.Get_otherphysicsitem(chrono.ChBody, 0),  
                  sys.Get_otherphysicsitem(fea.ChNodeFEAxyzrot, 0),
                  chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
sys.Add(motor)


def motor_speed(t):
    return 10  

motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed(0)))


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


vis_beam = fea.ChVisualizationFEAmesh(sys, True)
vis_beam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-0.1, 0.1)
vis_beam.SetSmoothFaces(True)
sys.Add(vis_beam)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'chrono_logo.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 1))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)