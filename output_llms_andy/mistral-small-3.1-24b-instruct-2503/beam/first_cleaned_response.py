import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fem as fem


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('PyChrono Beam FEM Simulation')
visualization.SetTryFullscreen(False)
visualization.Initialize()
visualization.AddLogo()
visualization.AddSkyBox()
visualization.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))


beam_mesh = fem.ChMesh()
beam_mesh.SetAutomaticGravity(True)


beam_material = chrono.ChMaterialShellBeam()
beam_material.SetYoungModulus(210e9)  
beam_material.SetPoissonRatio(0.3)    
beam_material.SetDensity(7800)        


num_elements = 10
length = 1.0
beam_elements = []

for i in range(num_elements):
    beam_element = fem.ChElementBeam()
    beam_element.SetMaterial(beam_material)
    beam_element.SetLength(length / num_elements)
    beam_element.SetCrossArea(0.01)  
    beam_element.SetJ(0.0001)        
    beam_element.SetE(210e9)         
    beam_element.SetG(80e9)          
    beam_element.SetRho(7800)        

    node1 = fem.ChNodeFEAxy()
    node1.SetPos(chrono.ChVectorD(i * length / num_elements, 0, 0))
    beam_mesh.AddNode(node1)

    node2 = fem.ChNodeFEAxy()
    node2.SetPos(chrono.ChVectorD((i + 1) * length / num_elements, 0, 0))
    beam_mesh.AddNode(node2)

    beam_element.AddNode(node1)
    beam_element.AddNode(node2)
    beam_mesh.AddElement(beam_element)
    beam_elements.append(beam_element)


system.Add(beam_mesh)


system.SetTimestepperType(chrono.ChTimestepper.HYBRID)
system.SetSolverType(chrono.ChSolver.MINRES)


while visualization.Run():
    system.DoStepDynamics(0.01)


visualization.Close()