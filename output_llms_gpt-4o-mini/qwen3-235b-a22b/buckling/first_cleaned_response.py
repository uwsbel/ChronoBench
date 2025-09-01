import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


L = 1.0              
radius = 0.02        
num_elements = 20    
num_nodes = num_elements + 1


mesh = fea.ChMesh()
nodes = []


for i in range(num_nodes):
    x = i * L / num_elements
    y = 0.0
    z = 0.0
    
    
    if i != 0:
        y += 0.001 * (chrono.ChRandom() - 0.5)
    
    pos = chrono.ChVectorD(x, y, z)
    node = fea.ChNodeFEAxyzrot(pos)
    if i == 0:
        node.SetFixed(True)  
    mesh.AddNode(node)
    nodes.append(node)


E = 2e5e6    
nu = 0.3     
area = chrono.CH_C_PI * radius**2
Iy = chrono.CH_C_PI * radius**4 / 4
Iz = Iy
G = E / (2 * (1 + nu))
J = Iy + Iz


for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    
    
    element.SetSectionArea(area)
    element.SetSectionIy(Iy)
    element.SetSectionIz(Iz)
    element.SetSectionYoungModulus(E)
    element.SetSectionShearModulus(G)
    element.SetSectionRadius(radius)
    
    mesh.AddElement(element)


system.Add(mesh)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


node_end = nodes[-1]
joint = chrono.ChLinkLockPrismatic()
joint.Initialize(ground, node_end, 
                chrono.ChCoordsysD(chrono.ChVectorD(L, 0, 0), 
                                   chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(joint)


motor = chrono.ChFunction_Ramp()
motor.SetCoeff(0.0, -0.02)  
joint.SetMotion_x(motor)


solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-8)
system.SetSolver(solver)

timestepper = chrono.ChTimestepperHHT(system)
timestepper.SetStepControl(True)
timestepper.SetAlpha(-0.1)
timestepper.SetMaxiters(20)
timestepper.SetAbsTolerances(1e-8)
timestepper.SetMode(chrono.ChTimestepperHHT.POSITION)
system.SetTimestepper(timestepper)


application = irr.ChIrrApp(system, 'Beam Buckling Simulation', irr.dimension2du(1024, 768))
application.AddLogo()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0.5, 0.5, 1.5))
application.AddTypicalLights()


vis_beam = fea.ChVisualizationFEAmesh(mesh)
vis_beam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
vis_beam.SetWireframe(True)
vis_beam.SetDrawInUndeformedReference(False)
mesh.AddAsset(vis_beam)


application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    application.DoStep()