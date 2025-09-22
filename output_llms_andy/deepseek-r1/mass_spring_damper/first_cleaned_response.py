import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    ground = chrono.ChBody()
    system.AddBody(ground)
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    
    
    ground_shape = chrono.ChBoxShape()
    ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(1, 0.1, 1)
    ground_shape.SetColor(chrono.ChColor(0.2, 0.3, 0.8))
    ground.AddAsset(ground_shape)
    
    
    mass = chrono.ChBody()
    system.AddBody(mass)
    mass.SetPos(chrono.ChVectorD(0, 2, 0))
    mass.SetMass(1.0)
    
    
    box_shape = chrono.ChBoxShape()
    box_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.2, 0.2)
    box_shape.SetColor(chrono.ChColor(0.8, 0.3, 0.2))
    mass.AddAsset(box_shape)
    
    
    spring = chrono.ChLinkTSDA()
    spring.Initialize(ground, mass, True, 
                      chrono.ChVectorD(0, 0, 0),  
                      chrono.ChVectorD(0, 0, 0))   
    spring.SetSpringCoefficient(50)    
    spring.SetDampingCoefficient(1)    
    spring.SetRestLength(2.0)          
    system.AddLink(spring)
    
    
    spring_shape = chrono.ChSpringShape(0.05, 100, 15)
    spring_shape.SetColor(chrono.ChColor(0.0, 0.8, 0.0))
    spring.AddAsset(spring_shape)
    
    
    vis = chronoirr.ChIrrApp(system, "Mass-Spring-Damper System", 800, 600)
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddTypicalLights(
        chronoirr.vector3df(-10, 10, -10),  
        chronoirr.vector3df(10, 10, 10)     
    )
    vis.AddCamera(chronoirr.vector3df(0, 3, -6))  
    
    
    vis.AssetBindAll()
    vis.AssetUpdateAll()
    
    
    vis.SetTimestep(0.01)
    vis.SetTryRealtime(True)
    
    
    while vis.GetDevice().run():
        vis.BeginScene()
        vis.DrawAll()
        vis.DoStep()
        vis.EndScene()

if __name__ == "__main__":
    main()