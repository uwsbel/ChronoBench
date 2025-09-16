import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


floor = chrono.ChBody()
floor.SetBodyFixed(True)
floor.SetName("floor")
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor_shape = chrono.ChBoxShape()
floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(5, 1, 5)
floor.AddAsset(floor_shape)
floor_texture = chrono.ChTexture()
floor_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
floor.AddAsset(floor_texture)
system.Add(floor)


crankshaft = chrono.ChBody()
crankshaft.SetMass(1.0)
crankshaft.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
cyl_shape = chrono.ChCylinderShape()
cyl_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.25, 0, 0)
cyl_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0.25, 0, 0)
cyl_shape.GetCylinderGeometry().rad = 0.05
crankshaft.AddAsset(cyl_shape)
crankshaft_color = chrono.ChColorAsset()
crankshaft_color.SetColor(chrono.ChColor(0.6, 0.3, 0.2))
crankshaft.AddAsset(crankshaft_color)
system.Add(crankshaft)


revolute_floor_crank = chrono.ChLinkLockRevolute()
revolute_floor_crank.Initialize(floor, crankshaft, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(revolute_floor_crank)


connecting_rod = chrono.ChBody()
connecting_rod.SetMass(0.5)
connecting_rod.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
connecting_rod.SetPos(chrono.ChVectorD(0.5, 0, 0))
rod_cyl = chrono.ChCylinderShape()
rod_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.25, 0, 0)
rod_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0.25, 0, 0)
rod_cyl.GetCylinderGeometry().rad = 0.025
connecting_rod.AddAsset(rod_cyl)
rod_color = chrono.ChColorAsset()
rod_color.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
connecting_rod.AddAsset(rod_color)
system.Add(connecting_rod)


revolute_crank_rod = chrono.ChLinkLockRevolute()
revolute_crank_rod.Initialize(crankshaft, connecting_rod, chrono.ChCoordsysD(chrono.ChVectorD(0.25, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(revolute_crank_rod)


piston = chrono.ChBody()
piston.SetMass(1.0)
piston.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
piston.SetPos(chrono.ChVectorD(0.75, 0, 0))
piston_box = chrono.ChBoxShape()
piston_box.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.1)
piston.AddAsset(piston_box)
piston_color = chrono.ChColorAsset()
piston_color.SetColor(chrono.ChColor(0.9, 0.9, 0.9))
piston.AddAsset(piston_color)
system.Add(piston)


revolute_rod_piston = chrono.ChLinkLockRevolute()
revolute_rod_piston.Initialize(connecting_rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(0.25, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(revolute_rod_piston)


prismatic_piston_floor = chrono.ChLinkLockPrismatic()
prismatic_piston_floor.Initialize(floor, piston, chrono.ChCoordsysD(chrono.ChVectorD(0.75, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(prismatic_piston_floor)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_C_PI))  
system.AddLink(motor)


vis = chronoirr.ChIrrApp(system, 'Crank-Slider Mechanism', chronoirr.dimension2du(1024, 768))
vis.AddTypicalLogo()
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.AddCamera(chronoirr.vector3df(2, 1, 3), chronoirr.vector3df(0, 0, 0))


vis.AssetBindAll()
vis.AssetUpdateAll()


while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)