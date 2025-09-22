import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    # 1. Initialize system and set gravity
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    # 2. Create ground body
    ground = chrono.ChBody()
    system.AddBody(ground)
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    
    # Add visualization asset for ground (flat box)
    ground_shape = chrono.ChBoxShape()
    ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(1, 0.1, 1)
    ground_shape.SetColor(chrono.ChColor(0.2, 0.3, 0.8))
    ground.AddAsset(ground_shape)
    
    # 3. Create mass body (box)
    mass = chrono.ChBody()
    system.AddBody(mass)
    mass.SetPos(chrono.ChVectorD(0, 2, 0))
    mass.SetMass(1.0)
    
    # Add visualization asset for mass
    box_shape = chrono.ChBoxShape()
    box_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.2, 0.2)
    box_shape.SetColor(chrono.ChColor(0.8, 0.3, 0.2))
    mass.AddAsset(box_shape)
    
    # 4. Create spring-damper between mass and ground
    spring = chrono.ChLinkTSDA()
    spring.Initialize(ground, mass, True, 
                      chrono.ChVectorD(0, 0, 0),  # Ground attachment point
                      chrono.ChVectorD(0, 0, 0))   # Mass attachment point
    spring.SetSpringCoefficient(50)    # N/m
    spring.SetDampingCoefficient(1)    # N/(m/s)
    spring.SetRestLength(2.0)          # Rest length = initial distance
    system.AddLink(spring)
    
    # Add spring visualization
    spring_shape = chrono.ChSpringShape(0.05, 100, 15)
    spring_shape.SetColor(chrono.ChColor(0.0, 0.8, 0.0))
    spring.AddAsset(spring_shape)
    
    # 5. Setup Irrlicht visualization
    vis = chronoirr.ChIrrApp(system, "Mass-Spring-Damper System", 800, 600)
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddTypicalLights(
        chronoirr.vector3df(-10, 10, -10),  # Light 1 position
        chronoirr.vector3df(10, 10, 10)     # Light 2 position
    )
    vis.AddCamera(chronoirr.vector3df(0, 3, -6))  # Camera position
    
    # Configure visualization system
    vis.AssetBindAll()
    vis.AssetUpdateAll()
    
    # 6. Simulation settings
    vis.SetTimestep(0.01)
    vis.SetTryRealtime(True)
    
    # 7. Run simulation
    while vis.GetDevice().run():
        vis.BeginScene()
        vis.DrawAll()
        vis.DoStep()
        vis.EndScene()

if __name__ == "__main__":
    main()