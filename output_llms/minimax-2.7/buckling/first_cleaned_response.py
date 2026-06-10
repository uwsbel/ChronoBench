import pychrono.core as chrono          
import pychrono.fea as chronofea        
import pychrono.irrlicht as irr          




system = chrono.ChSystemSMC()           
system.SetGravity(chrono.ChVectorD(0.0, 0.0, 0.0))   


solver = chrono.ChSolverMINRES()
system.SetSolver(solver)
system.SetSolverMaxIterations(500)      

timestepper = chrono.ChTimestepperHHT()
system.SetTimestepper(timestepper)


system.SetStep(1e-4)                    




mesh = chronofea.ChMesh()


L = 2.0                                   
num_nodes = 5
node_x = [i * L / (num_nodes - 1) for i in range(num_nodes)]

nodes = []
for x in node_x:
    
    node = chronofea.ChNodeFEAxyzD(chrono.ChVectorD(x, 0.0, 0.0))
    node.SetMass(0.0)                    
    mesh.AddNode(node)
    nodes.append(node)


nodes[0].Fix()


section = chronofea.ChBeamSectionCube()
section.SetWidth(0.05)                   
section.SetHeight(0.05)                 
section.SetYoungModulus(2.0e11)         
section.SetDensity(7800.0)              


for i in range(num_nodes - 1):
    elem = chronofea.ChElementBeamEuler()
    elem.SetNodes(nodes[i], nodes[i + 1])
    elem.SetSection(section)
    mesh.AddElement(elem)


system.Add(mesh)





ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0.0, 0.0, 0.0))
system.Add(ground)


end_mass = chrono.ChBody()
end_mass.SetPos(chrono.ChVectorD(L, 0.0, 0.0))
end_mass.SetMass(1.0)                  
system.Add(end_mass)



node_to_body = chronofea.ChLinkMateGeneric(True, True, True,
                                            True, True, True)   
node_to_body.Initialize(end_mass, nodes[-1])
system.Add(node_to_body)


actuator = chronofea.ChLinkLinActuator()
actuator.Initialize(ground, end_mass)                     
actuator.SetDirection(chrono.ChVectorD(1.0, 0.0, 0.0))    



motor_fun = chronofea.ChFunctionSequence()


ramp_up = chronofea.ChFunctionRamp()
ramp_up.Set_ang(0.0, 0.2)            
motor_fun.AddFun(ramp_up, 2.0)       


hold = chronofea.ChFunctionConst()
hold.Set_ang(0.4)                    
motor_fun.AddFun(hold, 3.0)          


ramp_down = chronofea.ChFunctionRamp()
ramp_down.Set_ang(0.4, -0.2)         
motor_fun.AddFun(ramp_down, 2.0)    

actuator.SetMotionFunction(motor_fun)
system.Add(actuator)




app = irr.ChIrrApp(system, 'Beam Buckling – PyChrono FEA',
                  irr.dimension2du(800, 600))

app.AddTypicalSkyBox()
app.AddTypicalCamera(irr.vector3df(0.0, 0.5, 3.0),
                     irr.vector3df(0.0, 0.0, 0.0))
app.AddTypicalLight()


app.AssetBindAll()
app.AssetUpdateAll()


app.SetStep(1e-4)




print("Running beam‑buckling simulation... (close Irrlicht window to stop)")
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()                     
    app.DoStep()                     
    app.EndScene()