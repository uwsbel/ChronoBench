import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


my_system = chrono.ChSystemNSC()
my_system.SetGravity(chrono.ChVectorD(0, 0, 0))  


beam_L = 1.0  
beam_radius = 0.05  
num_elements = 4  


msection = fea.ChBeamSectionEulerAdvanced()
msection.SetRadius(beam_radius)
msection.SetYoungModulus(2.1e11)  
msection.SetShearModulus(8.1e10)
msection.SetDensity(7800)
area = 3.14159 * beam_radius**2
I = 3.14159 * beam_radius**4 / 4
J = 2 * I  
msection.SetArea(area)
msection.SetIyy(I)
msection.SetIzz(I)
msection.SetJ(J)
msection.SetKsy(10)  
msection.SetKsz(10)


mesh = fea.ChMesh()
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, num_elements,
                  chrono.ChVectorD(0, 0, 0),
                  chrono.ChVectorD(beam_L, 0, 0),
                  chrono.ChVectorD(0, 1, 0))  

my_system.Add(mesh)


flywheel_mass = 10
flywheel_radius = 0.3
flywheel_length = 0.2  


flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVectorD(
    0.5 * flywheel_mass * flywheel_radius**2,  
    (flywheel_mass / 12) * (3 * flywheel_radius**2 + flywheel_length**2),  
    (flywheel_mass / 12) * (3 * flywheel_radius**2 + flywheel_length**2)   
))
flywheel.SetPos(chrono.ChVectorD(beam_L / 2, 0, 0))
flywheel.SetBodyFixed(False)
my_system.Add(flywheel)


center_node = None
for node in mesh.GetNodes():
    if abs(node.GetPos().x - beam_L / 2) < 1e-6:
        center_node = node
        break


if center_node:
    flywheel_link = fea.ChLinkNodeBody()
    flywheel_link.Initialize(center_node, flywheel)
    my_system.Add(flywheel_link)


end_node = mesh.GetNodes()[0]  


motor_body = chrono.ChBody()
motor_body.SetPos(end_node.GetPos())
motor_body.SetBodyFixed(False)
my_system.Add(motor_body)


motor_link = fea.ChLinkNodeBody()
motor_link.Initialize(end_node, motor_body)
my_system.Add(motor_link)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(motor_body, my_system.GetGround(), chrono.ChFrameD(end_node.GetPos()))
my_system.Add(motor)


speed_function = chrono.ChFunction_Const(10)
motor.SetSpeedFunction(speed_function)


application = irr.ChIrrApp(my_system, 'Jeffcott Rotor IGA Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 1, -2))


vis_shape = fea.ChVisualShapeFEA(mesh)
vis_shape.AddFEMglyphFEA(chrono.fea.ChGlyphType_GLYPH_ELEM_SHAFT, 0.01)
mesh.AddVisualShapeFEA(vis_shape)


flywheel_vis = chrono.ChVisualShapeCylinder(flywheel_radius, flywheel_length)
flywheel_vis.SetColor(chrono.ChColor(1, 0, 0))
flywheel.AddVisualShape(flywheel_vis, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_X))


motor_vis = chrono.ChVisualShapeSphere(0.05)
motor_vis.SetColor(chrono.ChColor(0, 0, 1))
motor_body.AddVisualShape(motor_vis)


application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    my_system.DoStepDynamics(application.GetTimestep())