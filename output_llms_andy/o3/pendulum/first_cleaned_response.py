import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())           
system = chrono.ChSystemNSC()                                  
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))               




time_step      = 1.0/500.0                                    
sim_end        = 10.0                                         
log_interval   = 0.1                                          
next_log_time  = 0.0


rod_length = 1.0                                              
rod_radius = 0.03                                             
rod_density= 1000                                             
rod_volume = math.pi*rod_radius**2 * rod_length
rod_mass   = rod_density * rod_volume
rod_inertia= chrono.ChVectorD(0.5*rod_mass*rod_radius**2,
                              0.5*rod_mass*rod_radius**2,
                              (1/2)*rod_mass*(rod_radius**2)) 




ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.02, 0.2)
ground.AddVisualShape(ground_shape)


system.Add(ground)




pendulum = chrono.ChBody()
pendulum.SetPos(chrono.ChVectorD(0, -rod_length/2.0, 0))      


pendulum.SetMass(rod_mass)
pendulum.SetInertiaXX(rod_inertia)


collision = chrono.ChCylinderShape()
collision.GetCylinderGeometry().p1 = chrono.ChVectorD(0,  rod_length/2.0, 0)
collision.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -rod_length/2.0, 0)
collision.GetCylinderGeometry().rad= rod_radius
pendulum.AddVisualShape(collision)                            


rot = chrono.Q_from_AngAxis(math.pi/2, chrono.ChVectorD(0,0,1))
pendulum.SetRot(rot)

system.Add(pendulum)





joint = chrono.ChLinkRevolute()
joint.Initialize(ground,                                   
                 pendulum,                                 
                 chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), 
                                   chrono.Q_from_AngAxis(math.pi/2,
                                                         chrono.ChVectorD(1,0,0)))) 
system.AddLink(joint)




app = chronoirr.ChIrrApp(system, "Simple pendulum", chronoirr.dimension2du(1024,768))
app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(1.5,0.0,1.5), chronoirr.vector3df(0,0,0))

app.AssetBindAll()
app.AssetUpdateAll()




while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()

    
    system.DoStepDynamics(time_step)

    
    t = system.GetChTime()
    if t >= next_log_time - 1e-9:
        tip_pos = pendulum.GetPos() + pendulum.GetRot().Rotate(chrono.ChVectorD(0, -rod_length/2.0, 0))
        tip_vel = pendulum.GetPos_dt() + pendulum.GetWvel_loc().Cross(chrono.ChVectorD(0, -rod_length/2.0, 0))
        print(f"[{t:6.3f} s]  tip pos = ({tip_pos.x:6.3f}, {tip_pos.y:6.3f}, {tip_pos.z:6.3f})  "
              f" tip vel = ({tip_vel.x:6.3f}, {tip_vel.y:6.3f}, {tip_vel.z:6.3f})")
        next_log_time += log_interval

    app.EndScene()

    
    if t > sim_end:
        break