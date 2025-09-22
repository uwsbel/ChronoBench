import pychrono as chrono
import pychrono.irrlicht as irr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())   
system          = chrono.ChSystemNSC()                 
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))        




L_crank   = 0.10          
L_rod     = 0.25          
crank_ω   = 2*chrono.CH_C_PI   
rod_rad   = 0.015         
piston_w  = 0.06          


ρ_steel   = 7800          
A_rod     = chrono.CH_C_PI * rod_rad**2
m_crank   = ρ_steel * (A_rod*L_crank)
m_rod     = ρ_steel * (A_rod*L_rod)
m_piston  = 1.00           




ground = chrono.ChBody()                     
ground.SetBodyFixed(True)
ground.SetName("Ground")


box = chrono.ChBoxShape()
box.GetBoxGeometry().Size = chrono.ChVectorD(0.15, 0.01, 0.15)
ground.AddAsset(box)
col = chrono.ChColorAsset()
col.SetColor(chrono.ChColor(0.3,0.3,0.3))
ground.AddAsset(col)

system.Add(ground)


shaft_pos = chrono.ChVectorD(0, 0, 0)       
slider_axis = chrono.ChVectorD(1, 0, 0)     




crank = chrono.ChBodyEasyCylinder(rod_rad, L_crank, ρ_steel)
crank.SetName("Crank")
crank.SetPos(shaft_pos + chrono.ChVectorD(L_crank*0.5, 0, 0))  
crank.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.ChVectorD(0,0,1)))
system.Add(crank)


rev_ground_crank = chrono.ChLinkLockRevolute()
rev_ground_crank.Initialize(crank, ground, chrono.ChCoordsysD(shaft_pos, chrono.Q_from_AngZ(0)))
system.Add(rev_ground_crank)




rod = chrono.ChBodyEasyCylinder(rod_rad, L_rod, ρ_steel)
rod.SetName("Rod")

rod_pin_B = shaft_pos + chrono.ChVectorD(L_crank, 0, 0)       
rod.SetPos((rod_pin_B + rod_pin_B + chrono.ChVectorD(L_rod,0,0))*0.5)  
rod.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.ChVectorD(0,0,1)))
system.Add(rod)


rev_crank_rod = chrono.ChLinkLockRevolute()
rev_crank_rod.Initialize(crank, rod,
                         chrono.ChCoordsysD(rod_pin_B,
                                            chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,0,1))))
system.Add(rev_crank_rod)




piston_shape = chrono.ChBoxShape()
piston_shape.GetBoxGeometry().Size = chrono.ChVectorD(piston_w*0.5, piston_w*0.5, piston_w*0.5)

piston = chrono.ChBody()
piston.SetName("Piston")
piston.SetMass(m_piston)
piston.SetInertiaXX(chrono.ChVectorD(1e-3,1e-3,1e-3))  
piston.AddAsset(piston_shape)


piston_pos = shaft_pos + chrono.ChVectorD(L_crank + L_rod, 0, 0)
piston.SetPos(piston_pos)
system.Add(piston)


rev_rod_piston = chrono.ChLinkLockRevolute()
rev_rod_piston.Initialize(rod, piston,
                          chrono.ChCoordsysD(piston_pos,
                                             chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,0,1))))
system.Add(rev_rod_piston)


slider = chrono.ChLinkLockPrismatic()
prismatic_frame = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),
                 chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.ChVectorD(0,0,1)))  
slider.Initialize(piston, ground, prismatic_frame)
system.Add(slider)




motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, ground, chrono.ChFrameD(shaft_pos, chrono.QUNIT))
fun_spd = chrono.ChFunction_Const(crank_ω)  
motor.SetSpeedFunction(fun_spd)
system.Add(motor)




vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono: Crank-Slider Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()                        
vis.AddCamera(chrono.ChVectorD(-0.4, 0.15, 0.4),   
              chrono.ChVectorD(0, 0, 0))            
vis.AddTypicalLights()
vis.BindAll()                         
vis.AddShadowAll()                    




time_step = 1e-3
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chrono.DrawAllMarkers(system)     
    vis.EndScene()

    system.DoStepDynamics(time_step)