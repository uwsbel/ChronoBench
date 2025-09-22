import pychrono as chrono
import pychrono.irrlicht as chronoirr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemNSC()




crank_radius = 0.1     
crank_length = 0.22    
rod_length = 0.4       
rod_radius = 0.04      
piston_width = 0.15    
piston_height = 0.12   
piston_depth = 0.12    
crank_omega = chrono.CH_C_PI  
floor_height = 0.05    


crank_center = chrono.ChVectorD(0, crank_radius + floor_height, 0)
slider_y = crank_center.y + rod_length






floor = chrono.ChBody()
floor.SetBodyFixed(True)
floor.SetPos(chrono.ChVectorD(0, 0, 0))
floor_shape = chrono.ChBoxShape()
floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(1.0, floor_height, 0.3)
floor.AddAsset(floor_shape)
floor_texture = chrono.ChTexture()
floor_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
floor.AddAsset(floor_texture)
system.Add(floor)


crank = chrono.ChBodyEasyCylinder(crank_radius, crank_length, 7800, True, True)
crank.SetPos(crank_center)
crank.SetRot(chrono.Q_from_AngZ(chrono.CH_C_PI/2))  
system.Add(crank)


rod = chrono.ChBodyEasyCylinder(rod_radius, rod_length, 7800, True, True)
rod.SetPos(chrono.ChVectorD(crank_center.x + crank_radius, crank_center.y + rod_length/2, 0))
rod.SetRot(chrono.Q_from_AngZ(chrono.CH_C_PI/2))  
system.Add(rod)


piston = chrono.ChBodyEasyBox(piston_width, piston_height, piston_depth, 7800, True, True)
piston.SetPos(chrono.ChVectorD(crank_center.x + crank_radius + rod_length, crank_center.y, 0))
piston.SetBodyFixed(False)
system.Add(piston)






rev_crank = chrono.ChLinkLockRevolute()
rev_crank.Initialize(crank, floor, chrono.ChCoordsysD(crank_center, chrono.Q_from_AngZ(chrono.CH_C_PI/2)))
system.AddLink(rev_crank)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFrameD(crank_center, chrono.Q_from_AngZ(chrono.CH_C_PI/2)))
motor_speed = chrono.ChFunction_Const(crank_omega)
motor.SetSpeedFunction(motor_speed)
system.AddLink(motor)


crank_pin_pos = crank_center + chrono.ChVectorD(crank_radius, 0, 0)
rev_crank_rod = chrono.ChLinkLockRevolute()
rev_crank_rod.Initialize(crank, rod, chrono.ChCoordsysD(crank_pin_pos, chrono.Q_from_AngZ(chrono.CH_C_PI/2)))
system.AddLink(rev_crank_rod)


rod_piston_pos = crank_pin_pos + chrono.ChVectorD(rod_length, 0, 0)
rev_rod_piston = chrono.ChLinkLockRevolute()
rev_rod_piston.Initialize(rod, piston, chrono.ChCoordsysD(rod_piston_pos, chrono.Q_from_AngZ(chrono.CH_C_PI/2)))
system.AddLink(rev_rod_piston)


prismatic = chrono.ChLinkLockPrismatic()
prismatic.Initialize(piston, floor, chrono.ChCoordsysD(piston.GetPos(), chrono.Q_from_AngY(chrono.CH_C_PI/2)))
system.AddLink(prismatic)






app = chronoirr.ChIrrApp(system, "Crank-Slider Mechanism", chronoirr.dimension2du(1024,768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(0.5, 0.5, 1.2), chronoirr.vector3df(0.5, 0.2, 0))  


crank.AddAsset(chrono.ChTexture(chrono.GetChronoDataFile("textures/bluewhite.png")))
rod.AddAsset(chrono.ChTexture(chrono.GetChronoDataFile("textures/greenwhite.png")))
piston.AddAsset(chrono.ChTexture(chrono.GetChronoDataFile("textures/redwhite.png")))

app.AssetBindAll()
app.AssetUpdateAll()




app.SetTimestep(0.002)
app.SetTryRealtime(True)

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()