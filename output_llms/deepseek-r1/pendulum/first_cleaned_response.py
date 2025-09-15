import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    system.Add(ground)
    
    
    ground_box = chrono.ChBoxShape()
    ground_box.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.1)
    ground_box.SetColor(chrono.ChColor(0.2, 0.3, 0.8))
    ground.AddAsset(ground_box)

    
    pendulum = chrono.ChBody()
    pendulum.SetPos(chrono.ChVectorD(0, -1, 0))  
    pendulum.SetMass(1.0)  
    
    
    length = 2.0  
    inertia = chrono.ChMatrix33D()
    inertia[0, 0] = 1  
    inertia[1, 1] = (1/12.0) * pendulum.GetMass() * (length**2)  
    inertia[2, 2] = 1  
    pendulum.SetInertia(inertia)
    
    
    pendulum_shape = chrono.ChBoxShape()
    pendulum_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.05, length/2, 0.05)
    pendulum_shape.SetColor(chrono.ChColor(0.9, 0.2, 0.2))
    pendulum.AddAsset(pendulum_shape)
    system.AddBody(pendulum)

    
    pivot = chrono.ChVectorD(0, 0, 0)  
    joint = chrono.ChLinkRevolute()
    joint.Initialize(ground, pendulum, 
                     chrono.ChCoordsysD(pivot, chrono.QUNIT))
    system.AddLink(joint)

    
    application = chronoirr.ChIrrApp(system, "Simple Pendulum", 800, 600)
    application.AddTypicalLogo()
    application.AddTypicalSky()
    application.AddTypicalLights()
    application.AddTypicalCamera(chronoirr.vector3df(0, -1, 3))
    application.AssetBindAll()
    application.AssetUpdateAll()

    
    time_step = 0.005
    application.SetTimestep(time_step)
    current_time = 0
    log_interval = 0.05  

    
    while application.GetDevice().run():
        application.BeginScene()
        application.DrawAll()
        application.DoStep()
        application.EndScene()
        
        
        current_time += time_step
        if current_time >= log_interval:
            current_time = 0
            pos = pendulum.GetPos()
            vel = pendulum.GetPos_dt()
            print(f"Time: {system.GetChTime():.3f}s | "
                  f"Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) | "
                  f"Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")

if __name__ == "__main__":
    main()