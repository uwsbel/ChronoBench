import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


system = chrono.ChSystemNSC()


floor = chrono.ChBody()
floor.SetBodyFixed(True)
floor.SetMass(0)
floor.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
floor.SetPos(chrono.ChVectorD(0, -0.1, 0))  
floor_shape = chrono.ChBoxShape()
floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(2, 0.1, 1)  
floor.AddAsset(floor_shape)
floor_texture = chrono.ChTexture()
floor_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
floor.AddAsset(floor_texture)
system.Add(floor)


crankshaft = chrono.ChBody()
crankshaft.SetMass(1)
crankshaft.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))  
crank_shape = chrono.ChBoxShape()
crank_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.05, 0.05)  
crankshaft.AddAsset(crank_shape)
crank_texture = chrono.ChTexture()
crank_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/blue.png"))
crankshaft.AddAsset(crank_texture)
system.Add(crankshaft)


connecting_rod = chrono.ChBody()
connecting_rod.SetMass(0.5)
connecting_rod.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
connecting_rod.SetPos(chrono.ChVectorD(0.35, 0, 0))  
rod_shape = chrono.ChBoxShape()
rod_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.25, 0.025, 0.025)  
connecting_rod.AddAsset(rod_shape)
rod_texture = chrono.ChTexture()
rod_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/red.png"))
connecting_rod.AddAsset(rod_texture)
system.Add(connecting_rod)


piston = chrono.ChBody()
piston.SetMass(2)
piston.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
piston.SetPos(chrono.ChVectorD(0.6, 0, 0))  
piston_shape = chrono.ChBoxShape()
piston_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.15, 0.05, 0.05)  
piston.AddAsset(piston_shape)
piston_texture = chrono.ChTexture()
piston_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/green.png"))
piston.AddAsset(piston_texture)
system.Add(piston)



revolute_floor_crank = chrono.ChLinkLockRevolute()
revolute_floor_crank.Initialize(floor, crankshaft, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.Add(revolute_floor_crank)


revolute_crank_rod = chrono.ChLinkLockRevolute()
revolute_crank_rod.Initialize(crankshaft, connecting_rod, chrono.ChCoordsysD(chrono.ChVectorD(0.1, 0, 0), chrono.QUNIT))
system.Add(revolute_crank_rod)


revolute_rod_piston = chrono.ChLinkLockRevolute()
revolute_rod_piston.Initialize(connecting_rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(0.6, 0, 0), chrono.QUNIT))
system.Add(revolute_rod_piston)


prismatic_piston_floor = chrono.ChLinkLockPrismatic()
prismatic_piston_floor.Initialize(floor, piston, chrono.ChCoordsysD(chrono.ChVectorD(0.6, 0, 0), chrono.QUNIT))
system.Add(prismatic_piston_floor)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(floor, crankshaft, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_C_PI * 2))  
system.Add(motor)


application = chronoirr.ChIrrApp(system, 'Crank-Slider Mechanism', chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1, 1, 2), chronoirr.vector3df(0, 0, 0))


application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()