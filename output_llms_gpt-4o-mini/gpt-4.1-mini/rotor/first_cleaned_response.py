import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())  
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  





beam_length = 1.0         
beam_divisions = 10       
beam_width = 0.02         
beam_height = 0.02        
E = 2.1e11                
rho = 7800                
nu = 0.3                  


mesh = fea.ChMesh()


section = fea.ChBeamSectionEulerAdvanced()
section.SetYoungModulus(E)
section.SetGfactor(E / (2*(1+nu)))
section.SetDensity(rho)
section.SetArea(beam_width * beam_height)
section.SetInertiaXX(  
    (beam_height*beam_width**3)/12, 
    (beam_width*beam_height**3)/12, 
    0)
section.SetWarpingConstant(beam_width * beam_height**3 / 12.0)  








nodes = []
for i in range(beam_divisions + 1):
    x = i * beam_length / beam_divisions
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(x, 0, 0)))
    node.SetNeumann(chrono.ChVectorD(0, 0, 0))  
    mesh.AddNode(node)
    nodes.append(node)





degree = 3
num_ctrl_pts = beam_divisions + 1
num_knots = num_ctrl_pts + degree + 1


knots = []
for i in range(degree + 1):
    knots.append(0.0)
for i in range(1, num_knots - 2 * (degree + 1) + 1):
    knots.append(i / (num_knots - 2 * (degree + 1) + 1))
for i in range(degree + 1):
    knots.append(1.0)
knots_arr = knots


basis = fea.ChContinuumBSpline(degree, knots_arr)









mesh.SetSplines([basis])


for i in range(beam_divisions):
    elem = fea.ChElementBeamIGA()
    elem.SetNodes(nodes[i], nodes[i + 1])
    elem.SetSection(section)
    elem.SetSpan(i)
    mesh.AddElement(elem)


system.Add(mesh)


nodes[0].SetFixed(True)




flywheel_radius = 0.1
flywheel_width = 0.02
flywheel_pos = chrono.ChVectorD(beam_length/2, 0, 0)

flywheel = chrono.ChBody()
flywheel.SetPos(flywheel_pos)
flywheel.SetMass(10)
flywheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.05))


flywheel_shape = chrono.ChCylinderShape()
flywheel_shape.GetCylinderGeometry().rad = flywheel_radius
flywheel_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -flywheel_width/2, 0)
flywheel_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, flywheel_width/2, 0)
flywheel.AddVisualShape(flywheel_shape)

system.Add(flywheel)



center_node = nodes[beam_divisions // 2]


joint = chrono.ChLinkMateGeneric()
joint.SetConstrainedCoords(True, True, True, True, True, True)
joint.SetFlipped(True)
joint.Initialize(flywheel, center_node, chrono.ChFrameD(flywheel_pos))
system.AddLink(joint)







system.RemoveLink(joint)





marker_node = chrono.ChBody()
marker_node.SetBodyFixed(True)
marker_node.SetPos(flywheel_pos)
system.Add(marker_node)


link_node = chrono.ChLinkPointFrame()
link_node.Initialize(center_node, marker_node)
system.AddLink(link_node)


motor_axis = chrono.ChVectorD(1, 0, 0)
rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(flywheel, marker_node, chrono.ChFrameD(flywheel_pos, chrono.Q_from_AngAxis(0, chrono.VECT_Z)))  
system.AddLink(rev_joint)










motor_base = chrono.ChBody()
motor_base.SetBodyFixed(True)
motor_base.SetPos(nodes[0].GetPos())
system.Add(motor_base)











beam_start = chrono.ChBody()
beam_start.SetBodyFixed(True)
beam_start.SetPos(nodes[0].GetPos())
system.Add(beam_start)



link_node0 = chrono.ChLinkPointFrame()
link_node0.Initialize(nodes[0], beam_start)
system.AddLink(link_node0)



motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam_start, system.Get_bodylist()[0], chrono.ChFrameD(beam_start.GetPos(), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_Y)))  
motor.SetSpeedFunction(chrono.ChFunction_Const(50))  
system.AddLink(motor)




application = chronoirr.ChIrrApp(system, "Jeffcott Rotor IGA Beam", chronoirr.dimension2du(1024, 768))

application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1.5, 0.3, 0.3))


vis_mymesh = fea.ChVisualizationMesh(mesh)
vis_mymesh.SetFEMdataType(fea.ChVisualizationMesh.E_PLOT_NODE_DISPLACEMENT)
vis_mymesh.SetColorscaleMinMax(-0.005, 0.005)
vis_mymesh.SetSmoothFaces(True)
mesh.AddVisualShape(vis_mymesh)



application.AssetBindAll()
application.AssetUpdateAll()


timestep = 1e-4

application.SetTimestep(timestep)

print("Starting simulation... Press Q or ESC to quit.")

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()