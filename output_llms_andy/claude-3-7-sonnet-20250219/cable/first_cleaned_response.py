import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


beam_length = 2.0       
beam_density = 1000     
beam_radius = 0.01      
num_elements = 10       
time_step = 1e-3        
simulation_time = 10    


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


beam_material = chrono.ChMaterialSurface()
beam_material.SetFriction(0.4)


beam_area = np.pi * beam_radius**2
beam_Iyy = np.pi * beam_radius**4 / 4
beam_Izz = beam_Iyy
E = 2e6  
G = 0.8e6  


beam_nodes = []
beam_elements = []
element_length = beam_length / num_elements


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


node0 = chrono.ChNodeFEAxyzD(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0))
node0.SetMass(0)  
beam_nodes.append(node0)
system.Add(node0)


constraint = chrono.ChLinkPointFrame()
constraint.Initialize(node0, ground)
system.Add(constraint)


for i in range(1, num_elements + 1):
    
    node_pos = chrono.ChVectorD(i * element_length, 0, 0)
    node_dir = chrono.ChVectorD(1, 0, 0)  
    node = chrono.ChNodeFEAxyzD(node_pos, node_dir)
    
    
    node.SetMass(beam_density * beam_area * element_length / 2)  
    beam_nodes.append(node)
    system.Add(node)
    
    
    element = chrono.ChElementCableANCF()
    element.SetNodes(beam_nodes[i-1], beam_nodes[i])
    element.SetDensity(beam_density)
    element.SetSection(beam_area)
    element.SetYoungModulus(E)
    element.SetShearModulus(G)
    element.SetIyy(Iyy=beam_Iyy)
    element.SetIzz(Izz=beam_Izz)
    
    beam_elements.append(element)
    system.Add(element)


application = chronoirr.ChIrrApp(system, "ANCF Cable Beam Demo", 
                                chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(1, 0.5, 2), chronoirr.vector3df(beam_length/2, 0, 0))
application.AddTypicalLights()
application.AddLightWithShadow(chronoirr.vector3df(1.5, 5.5, -2.5), 
                             chronoirr.vector3df(0, 0, 0), 
                             3, 2.2, 7.2, 40, 512)


for element in beam_elements:
    vis_shape = chrono.ChVisualShapeFEA(element.GetMesh())
    vis_shape.SetFEMdataType(chronoirr.ChVisualShapeFEA.DataType_BEAM)
    vis_shape.SetColorscale(0, 5)
    vis_shape.SetSmoothFaces(True)
    vis_shape.SetWireframe(False)
    element.AddVisualShapeFEA(vis_shape)


for node in beam_nodes:
    vis_node = chrono.ChVisualShapePointPoint(0.01, chrono.ChColor(0.2, 0.3, 0.4))
    node.AddVisualShape(vis_node)


ground_box = chrono.ChBoxShape(0.1, 0.1, 0.1)
ground.AddVisualShape(ground_box, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))


solver = chrono.ChSolverMINRES()
system.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)


system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
integrator = chrono.ChTimestepperHHT(system)
integrator.SetAlpha(-0.2)
integrator.SetMaxiters(5)
integrator.SetAbsTolerances(1e-05)
integrator.SetMode(chrono.ChTimestepperHHT.POSITION)
integrator.SetScaling(True)
integrator.SetVerbose(False)


application.AssetBindAll()
application.AssetUpdateAll()


system.SetChTime(0)
application.SetTimestep(time_step)


while application.GetDevice().run() and system.GetChTime() < simulation_time:
    
    application.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.2))
    application.DrawAll()
    
    
    if int(system.GetChTime() * 100) % 50 == 0:  
        print(f"Time: {system.GetChTime():.2f}")
        for i, node in enumerate(beam_nodes):
            pos = node.GetPos()
            print(f"  Node {i}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
    
    
    application.DoStep()
    application.EndScene()

print("Simulation completed")