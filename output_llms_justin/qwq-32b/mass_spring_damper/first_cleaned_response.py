import pychrono as chrono
import pychrono.irrlicht as irr

def main():
    
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

    
    ground = chrono.ChBody()
    my_system.Add(ground)
    ground.SetBodyFixed(True)
    ground.SetCollide(False)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))

    
    mass = chrono.ChBody()
    mass.SetMass(1.0)  
    mass.SetPos(chrono.ChVectorD(0, 0, 1))  

    
    box_size = chrono.ChVectorD(0.2, 0.2, 0.2)
    visual_box = chrono.ChVisualShapeBox(box_size.x, box_size.y, box_size.z)
    mass.AddAsset(visual_box)
    mass.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.8, 0, 0)))  

    
    inertia = chrono.ChMatrix33D()
    chrono.ChBoxInertia(inertia, mass.GetMass(), box_size.x, box_size.y, box_size.z)
    mass.SetInertia(inertia)

    my_system.Add(mass)

    
    link = chrono.ChLinkTSDA()
    link.Initialize(ground, mass, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
    link.SetRestLength(1.0)  
    link.Set_SpringK(100)    
    link.Set_SpringR(10)     

    
    spring_shape = chrono.ChSpringShape(link, 10, 0.02, 0.01)  
    link.AddAsset(spring_shape)
    my_system.Add(link)

    
    application = irr.ChIrrApp(my_system, 'Mass-Spring-Damper', irr.dimension2du(800, 600))
    application.AddTypicalLogo()
    application.AddTypicalSky()
    application.AddTypicalLights()
    application.AddTypicalCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0, 0, 1))

    
    application.AssetBind()
    application.AssetUpdate()

    
    application.SetTimestep(0.005)
    while application.GetDevice().run():
        application.BeginScene()
        application.DrawAll()
        application.DoStep()
        application.EndScene()

if __name__ == '__main__':
    main()