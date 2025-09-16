import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('./data/')
system = chrono.ChSystemNSC()
contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetDampingF(0.1)
contact_material.SetCompliance(1e-4)
contact_material.SetComplianceT(1e-4)


ground = chrono.ChBodyEasyBox(system, 10, 1, 10, 1000, True, contact_material)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


beam_length = 10
beam_thickness = 0.1
beam_density = 1000
beam_youngs_modulus = 1e9
beam_poissons_ratio = 0.3
beam_num_elements = 10


beam = chrono.ChBodyEasyCableANCF(beam_length, beam_thickness, beam_density, beam_youngs_modulus, beam_poissons_ratio, beam_num_elements, contact_material)
beam.SetPos(chrono.ChVectorD(0, 5, 0))


hinge = chrono.ChLinkHinge(beam, ground, chrono.ChVectorD(0, 5, 0), chrono.ChVectorD(0, -5, 0))
system.Add(hinge)


system.Add(beam)
system.Add(ground)


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetSolverType(chrono.ChSolver.Type_ACCGSM)


application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(1024, 768)
application.SetWindowTitle('Beam Simulation')
application.AddTypicalLights()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0))
application.SetShowFPS(True)


while application.Run():
    system.DoStepDynamics(0.01)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    
    for i in range(beam_num_elements):
        node_pos = beam.GetNodePosition(i)
        application.DrawSphere(chrono.ChVectorD(node_pos.x(), node_pos.y(), node_pos.z()), 0.1, chrono.ChVectorD(1, 0, 0))