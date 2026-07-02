import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


mesh = fea.ChMesh()


material = fea.ChIgaBeamSectionEulerAdvanced()
material.SetYoungModulus(210e9)  
material.SetGshearModulus(81e9)  
material.SetDensity(7800)  
material.SetAsCircularSection(0.02)  


control_points = [
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(0.5, 0, 0),
    chrono.ChVectorD(1.0, 0, 0),
    chrono.ChVectorD(1.5, 0, 0),
    chrono.ChVectorD(2.0, 0, 0)
]

knots_u = [0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1]


builder = fea.ChBuilderBeamIga()
builder.BuildBeam(mesh, material, len(control_points) - 1, control_points, knots_u)


system.Add(mesh)


center_node = None
for node in mesh.GetNodes():
    pos = node.GetPos()
    if abs(pos.x - 1.0) < 1e-6 and abs(pos.y) < 1e-6 and abs(pos.z) < 1e-6:
        center_node = node
        break


flywheel = chrono.ChBody()
flywheel.SetMass(1.0)  

Ixx = 0.5 * 1.0 * (0.1)**2  
flywheel.SetInertiaXX(chrono.ChVectorD(Ixx, Ixx, Ixx))
flywheel.SetPos(chrono.ChVectorD(1.0, 0, 0))
system.AddBody(flywheel)


link_node_flywheel = chrono.ChLinkNodeBody()
link_node_flywheel.Initialize(center_node, flywheel, chrono.ChFrameD(center_node.GetPos()))
system.AddLink(link_node_flywheel)


end_node = None
for node in mesh.GetNodes():
    pos = node.GetPos()
    if abs(pos.x - 2.0) < 1e-6 and abs(pos.y) < 1e-6 and abs(pos.z) < 1e-6:
        end_node = node
        break


driver = chrono.ChBody()
driver.SetBodyFixed(False)
system.AddBody(driver)


link_node_driver = chrono.ChLinkNodeBody()
link_node_driver.Initialize(end_node, driver, chrono.ChFrameD(end_node.GetPos()))
system.AddLink(link_node_driver)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(driver, system.GetGround(), chrono.ChFrameD(chrono.ChVectorD(2, 0, 0)))

motor.SetSpeedFunction(chrono.ChFunctionConst(10.0))
system.AddLink(motor)


fixed_body = chrono.ChBody()
fixed_body.SetBodyFixed(True)
system.AddBody(fixed_body)

first_node = None
for node in mesh.GetNodes():
    pos = node.GetPos()
    if abs(pos.x) < 1e-6 and abs(pos.y) < 1e-6 and abs(pos.z) < 1e-6:
        first_node = node
        break

link_node_fixed = chrono.ChLinkNodeBody()
link_node_fixed.Initialize(first_node, fixed_body, chrono.ChFrameD(first_node.GetPos()))
system.AddLink(link_node_fixed)


mesh.SetColor(chrono.ChColor(0.6, 0.6, 1.0))
mesh.SetDrawMode(fea.ChMeshDrawMode.E_PLOT_ELEM_BEAM_ABB)


application = irr.ChIrrApp(system, 'Jeffcott Rotor Simulation', irr.dimension2du(800, 600))
application.AddLogo()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 2, 5), chrono.ChVectorD(0, 0, 0))
application.AddTypicalLights()

application.AssetBindAll()
application.AssetUpdateAll()

application.SetTimestep(0.001)
application.SetTryRealtime(True)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()