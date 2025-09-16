import pychrono as chrono
import pychrono.irrlicht as irr

def main():
    
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
    ground = chrono.ChBody()
    my_system.Add(ground)
    ground.SetBodyFixed(True)
    ground.SetCollide(False)

    
    mass_size = 0.2
    mass_density = 125  
    mass = chrono.ChBodyEasyBox(mass_size, mass_size, mass_size, mass_density, True, True)
    mass.SetPos(chrono.ChVectorD(0, 1, 0))  
    mass.SetMass(1.0)
    mass.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0, 0))  
    my_system.Add(mass)

    
    springdamper = chrono.ChLinkTSDA()
    springdamper.Initialize(ground, mass, False, chrono.ChVectorD(0,0,0), chrono.ChVectorD(0,1,0))
    springdamper.SetRestLength(1.0)  
    springdamper.Set_SpringK(100)    
    springdamper.Set_SpringR(10)     
    my_system.AddLink(springdamper)

    
    spring_shape = chrono.ChSpringShape(springdamper, 20, 0.02)  
    springdamper.AddAsset(spring_shape)

    
    myapplication = irr.ChIrrApp(my_system, 'Mass-Spring-Damper Simulation', irr.dimension2du(1024,768))
    myapplication.AddTypicalLogo()
    myapplication.AddTypicalSky()
    myapplication.AddTypicalLights()
    myapplication.AddTypicalCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0,1,0))  

    
    myapplication.AssetReset()
    myapplication.SetTimestep(0.005)
    myapplication.SetTryRealtime(True)

    myapplication.Initialize()
    while myapplication.GetDevice().run():
        myapplication.BeginScene()
        myapplication.DrawAll()
        my_system.DoStepDynamics()
        myapplication.EndScene()

if __name__ == '__main__':
    main()