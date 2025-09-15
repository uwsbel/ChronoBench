import pychrono as chrono
import pychrono.irrlicht as irrlicht
import math

def main():
    
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    

    
    floor = chrono.ChBody()
    floor.SetBodyFixed(True)
    floor.SetPos(chrono.ChVectorD(0, -0.2, 0))
    floor_box = chrono.ChBoxShape()
    floor_box.GetBox().Size = chrono.ChVectorD(1, 0.1, 1)
    floor.AddAsset(floor_box)
    floor_texture = chrono.ChTexture(chrono.GetChronoDataFile("textures/checker2.png"))
    floor.AddAsset(floor_texture)
    system.Add(floor)

    
    crank = chrono.ChBodyEasyBox(0.2, 0.1, 0.1, 1000, True, True)
    crank.SetPos(chrono.ChVectorD(0, 0, 0))
    crank.SetRot(chrono.QUNIT)
    crank.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.8, 0.2, 0.2)))
    system.Add(crank)

    
    rod = chrono.ChBodyEasyBox(0.3, 0.05, 0.05, 1000, True, True)
    rod.SetPos(chrono.ChVectorD(0.25, 0, 0))
    rod.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.2, 0.8, 0.2)))
    system.Add(rod)

    
    piston = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
    piston.SetPos(chrono.ChVectorD(0.4, 0, 0))
    piston.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.2, 0.2, 0.8)))
    system.Add(piston)

    
    

    
    crank_joint = chrono.ChLinkLockRevolute()
    crank_joint.Initialize(crank, floor, 
                          chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                                           chrono.Q_from_AngAxis(math.pi/2, chrono.VECT_Y)))
    system.Add(crank_joint)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(crank, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    motor.SetSpeedFunction(chrono.ChFunction_Const(math.pi))  
    system.Add(motor)

    
    rod_joint1 = chrono.ChLinkLockRevolute()
    rod_joint1.Initialize(crank, rod, 
                         chrono.ChCoordsysD(chrono.ChVectorD(0.1, 0, 0), chrono.QUNIT))
    system.Add(rod_joint1)

    
    rod_joint2 = chrono.ChLinkLockRevolute()
    rod_joint2.Initialize(rod, piston, 
                         chrono.ChCoordsysD(chrono.ChVectorD(0.4, 0, 0), chrono.QUNIT))
    system.Add(rod_joint2)

    
    piston_joint = chrono.ChLinkLockPrismatic()
    piston_joint.Initialize(piston, floor, 
                           chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    system.Add(piston_joint)

    
    

    
    myapplication = irrlicht.ChIrrApp(system, "Crank-Slider Mechanism", irrlicht.dimension2du(1024,768))
    myapplication.AddTypicalSky()
    myapplication.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    myapplication.AddTypicalCamera(irrlicht.vector3df(1.5, 0.5, 1.5), irrlicht.vector3df(0,0,0))
    myapplication.AddTypicalLights()
    myapplication.EnableShadows()

    
    myapplication.AssetBindAll()
    myapplication.AssetUpdateAll()

    
    myapplication.SetTimestep(0.005)
    myapplication.SetTryRealtime(True)

    while myapplication.GetDevice().run():
        myapplication.BeginScene()
        myapplication.DrawAll()
        irrlicht.ChIrrTools.drawAllCOGs(system, myapplication.GetVideoDriver(), 0.5)
        myapplication.DoStep()
        myapplication.EndScene()

if __name__ == "__main__":
    main()