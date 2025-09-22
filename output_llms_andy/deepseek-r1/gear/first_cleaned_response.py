import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  

    
    steel = chrono.ChMaterialSurfaceNSC()
    steel.SetFriction(0.1)
    steel.SetRestitution(0.1)

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    
    
    sun_radius = 0.5
    sun = chrono.ChBodyEasyCylinder(sun_radius, 0.1, 1000, True, True, steel)
    sun.SetPos(chrono.ChVectorD(0, 0, 0))
    sun.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI_2))
    sun.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    system.Add(sun)
    
    
    sun_revolute = chrono.ChLinkLockRevolute()
    sun_revolute.Initialize(sun, ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
    system.Add(sun_revolute)

    
    carrier = chrono.ChBodyEasyBox(0.1, 1.0, 0.1, 1000, True, True, steel)
    carrier.SetPos(chrono.ChVectorD(0, 0, 0))
    carrier.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.8, 0.2))
    system.Add(carrier)
    
    
    carrier_revolute = chrono.ChLinkLockRevolute()
    carrier_revolute.Initialize(carrier, ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
    system.Add(carrier_revolute)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(carrier, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI / 3))  
    system.Add(motor)

    
    planet_radius = 0.2
    planet = chrono.ChBodyEasyCylinder(planet_radius, 0.1, 1000, True, True, steel)
    planet.SetPos(chrono.ChVectorD(0, 0.7, 0))
    planet.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI_2))
    planet.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.8))
    system.Add(planet)
    
    
    planet_revolute = chrono.ChLinkLockRevolute()
    planet_revolute.Initialize(planet, carrier, 
                              chrono.ChCoordsysD(chrono.ChVectorD(0, 0.7, 0)))
    system.Add(planet_revolute)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Epicyclic Gears')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1, 0.5, 1))
    vis.AddTypicalLights()

    
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()