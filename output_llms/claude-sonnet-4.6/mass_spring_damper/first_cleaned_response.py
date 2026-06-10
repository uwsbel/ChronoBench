import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math






system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))






contact_material = chrono.ChContactMaterialNSC()
contact_material.SetFriction(0.5)






ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
system.AddBody(ground)


ground_shape = chrono.ChVisualShapeBox(4.0, 0.2, 4.0)
ground_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(ground_shape, chrono.ChFramed(chrono.ChVector3d(0, -0.1, 0)))


ground_attach_shape = chrono.ChVisualShapeCylinder(0.05, 0.3)
ground_attach_shape.SetColor(chrono.ChColor(0.6, 0.3, 0.1))
ground.AddVisualShape(ground_attach_shape, 
                      chrono.ChFramed(chrono.ChVector3d(0, 2.0, 0),
                                      chrono.QuatFromAngleX(math.pi / 2)))






mass_value = 2.0         
spring_rest_length = 1.5 
initial_offset = 0.5     


mass_body = chrono.ChBody()
mass_body.SetMass(mass_value)
mass_body.SetPos(chrono.ChVector3d(0, spring_rest_length + initial_offset, 0))


side = 0.3
inertia = (1.0 / 6.0) * mass_value * side * side
mass_body.SetInertiaXX(chrono.ChVector3d(inertia, inertia, inertia))

system.AddBody(mass_body)


mass_shape = chrono.ChVisualShapeBox(side, side, side)
mass_shape.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
mass_body.AddVisualShape(mass_shape)






spring_coeff   = 50.0   
damping_coeff  = 5.0    
rest_length    = spring_rest_length  


ground_attach_point = chrono.ChVector3d(0, 2.0, 0)   
mass_attach_point   = chrono.ChVector3d(0, 0, 0)      


spring_damper = chrono.ChLinkTSDA()
spring_damper.SetSpringCoefficient(spring_coeff)
spring_damper.SetDampingCoefficient(damping_coeff)
spring_damper.SetRestLength(rest_length)


spring_damper.Initialize(
    ground,                    
    mass_body,                 
    True,                      
    ground_attach_point,       
    mass_body.GetPos()         
)

system.AddLink(spring_damper)


spring_visual = chrono.ChVisualShapeSpring(0.1, 80, 15)
spring_visual.SetColor(chrono.ChColor(0.9, 0.2, 0.2))
spring_damper.AddVisualShape(spring_visual)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Mass-Spring-Damper System (ChLinkTSDA)")
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()


vis.AddCamera(
    chrono.ChVector3d(3.0, 3.0, 5.0),   
    chrono.ChVector3d(0.0, 2.0, 0.0)    
)


vis.AddTypicalLights()
vis.AddLight(
    chrono.ChVector3d(4.0, 8.0, 4.0),   
    10.0,                                 
    chrono.ChColor(1.0, 1.0, 1.0)        
)






time_step = 0.001   
end_time  = 10.0    


print("=" * 60)
print(f"{'Time':>8}  {'Pos_Y':>10}  {'Vel_Y':>10}  {'Spring_F':>12}")
print("=" * 60)


time = 0.0
frame_count = 0

while vis.Run() and time < end_time:
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if frame_count % 100 == 0:
        pos_y    = mass_body.GetPos().y
        vel_y    = mass_body.GetPosDt().y
        spring_f = spring_damper.GetForce()
        print(f"{time:>8.3f}  {pos_y:>10.4f}  {vel_y:>10.4f}  {spring_f:>12.4f}")

    
    system.DoStepDynamics(time_step)

    time += time_step
    frame_count += 1

print("=" * 60)
print("Simulation complete.")