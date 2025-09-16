import pychrono as chrono
import pychrono.irrlicht as irr

def main():
    # Create the Chrono system with gravity along the negative Y-axis
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # Create ground body (static)
    ground = chrono.ChBody()
    my_system.Add(ground)
    ground.SetBodyFixed(True)
    ground.SetCollide(False)

    # Create mass body with dimensions 0.2m x 0.2m x 0.2m
    mass_size = 0.2
    mass_density = 125  # To achieve a mass of 1 kg (volume * density = 0.008 * 125 = 1)
    mass = chrono.ChBodyEasyBox(mass_size, mass_size, mass_size, mass_density, True, True)
    mass.SetPos(chrono.ChVectorD(0, 1, 0))  # Start at (0,1,0) along Y-axis
    mass.SetMass(1.0)
    mass.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0, 0))  # Red color for visibility
    my_system.Add(mass)

    # Create spring-damper link between ground and mass
    springdamper = chrono.ChLinkTSDA()
    springdamper.Initialize(ground, mass, False, chrono.ChVectorD(0,0,0), chrono.ChVectorD(0,1,0))
    springdamper.SetRestLength(1.0)  # Initial length between points
    springdamper.Set_SpringK(100)    # Spring constant (N/m)
    springdamper.Set_SpringR(10)     # Damping coefficient (Ns/m)
    my_system.AddLink(springdamper)

    # Add visual spring shape to the link
    spring_shape = chrono.ChSpringShape(springdamper, 20, 0.02)  # 20 coils, radius 0.02m
    springdamper.AddAsset(spring_shape)

    # Initialize Irrlicht visualization
    myapplication = irr.ChIrrApp(my_system, 'Mass-Spring-Damper Simulation', irr.dimension2du(1024,768))
    myapplication.AddTypicalLogo()
    myapplication.AddTypicalSky()
    myapplication.AddTypicalLights()
    myapplication.AddTypicalCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0,1,0))  # Camera position and target

    # Simulation settings and run loop
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