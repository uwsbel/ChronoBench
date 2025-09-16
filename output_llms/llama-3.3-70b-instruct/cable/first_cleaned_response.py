import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.analytical_functions as chronoaf


sys = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetRestitution(0.1)
contact_material.SetCompliance(1e-6)


beam_length = 10  
num_elements = 10  
density = 7850  
youngs_modulus = 2e11  
poissons_ratio = 0.3  


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, contact_material)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


beam = chrono.ChBody()
beam.SetMass(1)
beam.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
beam.SetInertiaXY(chrono.ChVectorD(0, 0, 0))
beam.SetInertiaXZ(chrono.ChVectorD(0, 0, 0))
beam.SetInertiaYY(chrono.ChVectorD(1, 1, 1))
beam.SetInertiaYZ(chrono.ChVectorD(0, 0, 0))
beam.SetInertiaZZ(chrono.ChVectorD(1, 1, 1))


ancf_elements = []
for i in range(num_elements):
    ancf_element = chrono.ChElementCableANCF(beam_length / num_elements, density, youngs_modulus, poissons_ratio)
    ancf_elements.append(ancf_element)
    beam.Add(ancf_element)


hinge = chrono.ChLinkMateGeneric()
hinge.SetName("hinge")
hinge.Initialize(beam, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))


sys.Add(hinge)


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.AddTypicalLights()


time_step = 0.01
end_time = 10
while sys.GetChTime() < end_time:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.Run()

    
    for i, ancf_element in enumerate(ancf_elements):
        print(f"Node {i+1} position: {ancf_element.GetNode().GetPos()}")