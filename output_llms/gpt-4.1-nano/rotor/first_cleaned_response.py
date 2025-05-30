import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()






beam_length = 1.0  
beam_radius = 0.02  
beam_density = 7800  
beam_E = 2.1e11      
beam_area = 3.14159 * beam_radius**2
beam_inertia = (chrono.ChVectorD(0, 0, beam_radius**4 * 3.14159 / 4),
                chrono.ChVectorD(0, 0, beam_radius**4 * 3.14159 / 4))  


start_pos = chrono.ChVectorD(0, 0, 0)
end_pos = chrono.ChVectorD(beam_length, 0, 0)





beam_body = chrono.ChBody()
beam_body.SetMass(beam_density * beam_area * beam_length)
beam_inertia_diag = beam_body.GetInertia()

beam_body.SetInertiaXX(chrono.ChVectorD(
    (1/12)*beam_body.GetMass()*(beam_length**2),
    (1/12)*beam_body.GetMass()*(beam_radius**2),
    (1/12)*beam_body.GetMass()*(beam_radius**2)
))


beam_body.SetPos(start_pos + chrono.ChVectorD(beam_length/2, 0, 0))
system.Add(beam_body)








flywheel_radius = 0.05
flywheel_mass = 2.0  

flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVectorD(
    0.5 * flywheel_mass * flywheel_radius**2,
    0.5 * flywheel_mass * flywheel_radius**2,
    0.5 * flywheel_mass * flywheel_radius**2
))

mid_pos = chrono.ChVectorD(beam_length/2, 0, 0)
flywheel.SetPos(mid_pos)
system.Add(flywheel)



joint_flywheel = chrono.ChLinkLockRevolute()
joint_flywheel.Initialize(beam_body, flywheel, chrono.CrdTransform(mid_pos, chrono.QUNIT))
system.Add(joint_flywheel)







fixed_point = chrono.ChVectorD(0, 0, 0)


end_body = chrono.ChBody()
end_body.SetMass(0)  
end_body.SetPos(start_pos)
system.Add(end_body)


revolute_joint = chrono.ChLinkMotorRotationSpeed()
revolute_joint.Initialize(end_body, system.Get_ground(), chrono.ChFrameD(fixed_point, chrono.Q_from_AngZ(0)))
system.Add(revolute_joint)


motor_speed = 100.0
revolute_joint.SetSpeed(motor_speed)



end_node = chrono.ChNodeShape_Utils()  





beam_end_body = chrono.ChBody()
beam_end_body.SetPos(end_pos)
system.Add(beam_end_body)

joint_end = chrono.ChLinkRevolute()
joint_end.Initialize(beam_body, beam_end_body, chrono.CrdTransform(end_pos, chrono.QIDENT))
system.Add(joint_end)


joint_motor = chrono.ChLinkMotorRotationSpeed()
joint_motor.Initialize(end_body, system.Get_ground(), chrono.CrdTransform(end_pos, chrono.Q_from_AngZ(0)))
system.Add(joint_motor)
joint_motor.SetSpeed(motor_speed)






application = irr.ChVisualSystemIrrlicht()
application.AttachSystem(system)
application.SetWindowSize(800, 600)
application.SetWindowTitle('Jeffcott Rotor Simulation')
application.Initialize()



beam_visual = irr.ChCylinderShape(beam_radius, beam_length)
beam_visual.SetColor(irr.WithAlpha(irr.C_color(0.3, 0.3, 0.8), 0.8))
application.GetVideoDriver().BindAll()


flywheel_visual = irr.ChSphereShape(flywheel_radius)
flywheel_visual.SetColor(irr.C_color(0.8, 0.1, 0.1))
application.GetSceneManager().AddSphere(flywheel_visual, True, flywheel.GetPos(), chrono.ChQuaternionD(1,0,0,0))



while application.GetDevice().run():
    system.DoStepDynamics(1e-3)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()


application.GetDevice().close()