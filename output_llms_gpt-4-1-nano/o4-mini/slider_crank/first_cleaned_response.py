import pychrono.core as chrono
import pychrono.irrlicht as chronoirr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())  
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




floor = chrono.ChBodyEasyBox(2.0, 0.1, 0.5,    
                             1000,              
                             True, True)        
floor.SetPos(chrono.ChVectorD(0, -0.05, 0))
floor.SetBodyFixed(True)
system.Add(floor)

floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataPath() + 'textures/concrete.jpg')




crank_radius    = 0.2
crank_thickness = 0.02
crank_density   = 1000

crank = chrono.ChBodyEasyCylinder(crank_radius, crank_thickness,
                                  crank_density, True, True)

crank.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(crank)


rev_crank = chrono.ChLinkLockRevolute()

rev_crank.Initialize(crank,    
                     system.Get_bodylist()[0],  
                     chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),
                                       chrono.Q_from_AngAxis(0,0,1,0)))
system.Add(rev_crank)


motor = chrono.ChLinkMotorRotationSpeed()
motor_speed = chrono.ChFunction_Const(chrono.CH_C_PI)  
motor.Initialize(crank, system.Get_bodylist()[0],
                 chrono.ChFrameD(chrono.ChVectorD(0,0,0),
                                 chrono.Q_from_AngAxis(0,0,1,0)))
motor.SetSpeedFunction(motor_speed)
system.Add(motor)




rod_length  = 1.0
rod_radius  = 0.05
rod_density = 1000

rod = chrono.ChBodyEasyCylinder(rod_radius, rod_length,
                                rod_density, True, True)

rod.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.ChVectorD(0,1,0)))

initial_crank_pin = chrono.ChVectorD(crank_radius, 0, 0)
initial_piston_x  = crank_radius + rod_length
rod_center = chrono.ChVectorD( (crank_radius + initial_piston_x)/2, 0, 0 )
rod.SetPos(rod_center)
system.Add(rod)


rev_cr_rod = chrono.ChLinkLockRevolute()
rev_cr_rod.Initialize(crank, rod,
    chrono.ChCoordsysD(initial_crank_pin,
                      chrono.Q_from_AngAxis(0,0,1,0)))
system.Add(rev_cr_rod)




piston = chrono.ChBodyEasyBox(0.2, 0.2, 0.2,  
                              1000,          
                              True, True)    
piston.SetPos(chrono.ChVectorD(initial_piston_x, 0, 0))
system.Add(piston)

prism = chrono.ChLinkLockPrismatic()

prism.Initialize(piston, system.Get_bodylist()[0],
                 chrono.ChCoordsysD(chrono.ChVectorD(0,0,0),
                                   chrono.Q_from_AngAxis(0,1,0,0)))
system.Add(prism)


rod_end = chrono.ChVectorD(initial_piston_x, 0, 0)
rev_rod_piston = chrono.ChLinkLockRevolute()
rev_rod_piston.Initialize(rod, piston,
    chrono.ChCoordsysD(rod_end, chrono.Q_from_AngAxis(0,0,1,0)))
system.Add(rev_rod_piston)




app = chronoirr.ChIrrApp(system, "Crank–Slider Mechanism",
                         chronoirr.dimension2du(1024, 768))
app.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")  
app.AddSkyBox()
app.AddTypicalLights()

app.AddCamera(chronoirr.vector3df(1.5, 1.5, 1.5),
              chronoirr.vector3df(0, 0, 0))
app.AssetBindAll()
app.AssetUpdateAll()




timestep = 1e-3
while app.Run():
    app.BeginScene()
    app.DrawAll()
    system.DoStepDynamics(timestep)
    app.EndScene()