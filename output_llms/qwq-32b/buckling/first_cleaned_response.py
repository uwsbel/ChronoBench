import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


my_system = chrono.ChSystemMulticore()
fea_system = fea.ChFEMintegrate(my_system)


vis = irr.ChIrrApp(my_system, 'Beam Buckling Simulation', irr.dimension2du(800,600))
vis.AddFEAMesh(fea_system)
vis.SetCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0,0,0))
vis.SetSymbolscale(0.001)
vis.SetShadowAll(0)
vis.SetVideoframeCloud(0)
vis.SetVideoframeSpheresize(0.001)
vis.SetVideoframeLinesize(0.0005)


beam_length = 2.0
num_elements = 20
num_nodes = num_elements + 1
cross_section_width = 0.1  
cross_section_height = 0.02  


young_modulus = 2.1e9  
poisson_ratio = 0.3
density = 7850  


nodes = []
for i in range(num_nodes):
    x = i * beam_length / num_elements
    node = fea.ChNodeFEAxyzD()
    node.SetPos(chrono.ChVectorD(x, 0, 0))
    fea_system.AddNode(node)
    nodes.append(node)


nodes[0].SetFixed(True)


for i in range(num_elements):
    elem = fea.ChElementBeamANCF()
    elem.SetNodes(nodes[i], nodes[i+1])
    elem.Set_E(young_modulus)
    elem.Set_nu(poisson_ratio)
    elem.Set_rho(density)
    
    area = cross_section_width * cross_section_height
    Ixx = (cross_section_width * cross_section_height**3) / 12
    Iyy = (cross_section_height * cross_section_width**3) / 12
    elem.Set_A(area)
    elem.Set_Ixx(Ixx)
    elem.Set_Iyy(Iyy)
    fea_system.AddElement(elem)


class CustomForceFunction(chrono.ChFunction):
    def Get_y(self, time):
        
        return -1e4 * time  

force_func = CustomForceFunction()
force = fea.ChLoadBodyForce()
force.SetLoadDir(chrono.ChVectorD(-1, 0, 0))  
force.SetF(force_func)
force.SetNode(nodes[-1])
fea_system.AddLoad(force)


integrator = fea.ChFEMintegrateImplicit()
integrator.SetTolerance(1e-6)
integrator.SetMaxIterations(50)
fea_system.SetIntegrator(integrator)


my_system.SetTimestep(0.001)


vis.SetTimestep(0.001)
vis.SetTryRealtime(True)

while vis.Run():
    my_system.DoStepDynamics(0.001)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


my_system = chrono.ChSystemMulticore()
fea_system = fea.ChFEMintegrate(my_system)


vis = irr.ChIrrApp(my_system, 'Beam Buckling Simulation', irr.dimension2du(800, 600))
vis.AddFEAMesh(fea_system)
vis.SetCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0, 0, 0))
vis.SetSymbolscale(0.001)
vis.SetShadowAll(0)
vis.SetVideoframeCloud(0)
vis.SetVideoframeSpheresize(0.001)
vis.SetVideoframeLinesize(0.0005)


beam_length = 2.0
num_elements = 20
num_nodes = num_elements + 1
cross_section_width = 0.1  
cross_section_height = 0.02  


young_modulus = 2.1e9  
poisson_ratio = 0.3
density = 7850  


nodes = []
for i in range(num_nodes):
    x = i * beam_length / num_elements
    node = fea.ChNodeFEAxyzD()
    node.SetPos(chrono.ChVectorD(x, 0, 0))
    fea_system.AddNode(node)
    nodes.append(node)


nodes[0].SetFixed(True)


for i in range(num_elements):
    elem = fea.ChElementBeamANCF()
    elem.SetNodes(nodes[i], nodes[i+1])
    elem.Set_E(young_modulus)
    elem.Set_nu(poisson_ratio)
    elem.Set_rho(density)
    
    
    area = cross_section_width * cross_section_height
    Ixx = (cross_section_width * cross_section_height**3) / 12
    Iyy = (cross_section_height * cross_section_width**3) / 12
    elem.Set_A(area)
    elem.Set_Ixx(Ixx)
    elem.Set_Iyy(Iyy)
    
    fea_system.AddElement(elem)


class CustomForceFunction(chrono.ChFunction):
    def Get_y(self, time):
        return -1e4 * time  

force_func = CustomForceFunction()
force = fea.ChLoadBodyForce()
force.SetLoadDir(chrono.ChVectorD(-1, 0, 0))  
force.SetF(force_func)
force.SetNode(nodes[-1])
fea_system.AddLoad(force)


integrator = fea.ChFEMintegrateImplicit()
integrator.SetTolerance(1e-6)
integrator.SetMaxIterations(50)
fea_system.SetIntegrator(integrator)


my_system.SetTimestep(0.001)
vis.SetTimestep(0.001)
vis.SetTryRealtime(True)


while vis.Run():
    my_system.DoStepDynamics(0.001)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()