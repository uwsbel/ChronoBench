import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(2, 0.1, 1)
ground.AddVisualShape(ground_shape, chrono.ChFrameD(chrono.ChVectorD(0, -0.1, 0)))


ground_material = chrono.ChVisualMaterial()
ground_material.SetDiffuseColor(chrono.ChColor(0.3, 0.3, 0.3))
ground_shape.SetMaterial(0, ground_material)


mass_body = chrono.ChBody()
mass_body.SetMass(1.0)  
mass_body.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  
mass_body.SetPos(chrono.ChVectorD(0, 2, 0))  
mass_body.SetPos_dt(chrono.ChVectorD(0, 0, 0))  
mass_body.SetBodyFixed(False)
system.Add(mass_body)


mass_shape = chrono.ChSphereShape()
mass_shape.GetSphereGeometry().rad = 0.2
mass_body.AddVisualShape(mass_shape)


mass_material = chrono.ChVisualMaterial()
mass_material.SetDiffuseColor(chrono.ChColor(0.8, 0.2, 0.2))
mass_shape.SetMaterial(0, mass_material)


spring_damper = chrono.ChLinkTSDA()



ground_point = chrono.ChVectorD(0, 0, 0)

mass_point = chrono.ChVectorD(0, 0, 0)


spring_damper.Initialize(ground, mass_body, False, ground_point, mass_point)


rest_length = 1.5  
spring_constant = 50.0  
damping_coefficient = 2.0  

spring_damper.SetRestLength(rest_length)
spring_damper.SetSpringCoefficient(spring_constant)
spring_damper.SetDampingCoefficient(damping_coefficient)


system.Add(spring_damper)


spring_visual = chrono.ChVisualShapeSegment()
spring_damper.AddVisualShape(spring_visual)


spring_visual_material = chrono.ChVisualMaterial()
spring_visual_material.SetDiffuseColor(chrono.ChColor(0.2, 0.8, 0.2))
spring_visual.SetMaterial(0, spring_visual_material)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(4, 3, 4), chrono.ChVectorD(0, 1, 0))


vis.AddTypicalLights()


time_step = 0.01
simulation_time = 10.0
current_time = 0


while current_time < simulation_time:
    
    if not vis.Run():
        break
    
    
    vis.BeginScene()
    vis.Render()
    
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Time: {current_time:.2f}s",
        chronoirr.irr.core.rect(10, 10, 200, 30),
        chronoirr.irr.video.SColor(255, 255, 255, 255)
    )
    
    mass_pos = mass_body.GetPos()
    spring_length = spring_damper.GetLength()
    spring_force = spring_damper.GetSpringForce()
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Mass Y-pos: {mass_pos.y:.3f}m",
        chronoirr.irr.core.rect(10, 30, 250, 50),
        chronoirr.irr.video.SColor(255, 255, 255, 255)
    )
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Spring Length: {spring_length:.3f}m",
        chronoirr.irr.core.rect(10, 50, 250, 70),
        chronoirr.irr.video.SColor(255, 255, 255, 255)
    )
    
    vis.GetGUIEnvironment().getBuiltInFont().draw(
        f"Spring Force: {spring_force:.3f}N",
        chronoirr.irr.core.rect(10, 70, 250, 90),
        chronoirr.irr.video.SColor(255, 255, 255, 255)
    )
    
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)
    current_time += time_step


vis.GetDevice().closeDevice()