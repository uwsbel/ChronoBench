import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    system.Add(ground)

    
    pendulum = chrono.ChBodyEasyBox(0.1, 2.0, 0.1,  
                                    1000,            
                                    True, True)      
    pendulum.SetPos(chrono.ChVectorD(0, -1, 0))      
    pendulum.SetRot(chrono.QUNIT)                    
    system.Add(pendulum)

    
    revolute = chrono.ChLinkLockRevolute()
    revolute.Initialize(ground, 
                        pendulum, 
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),  
                                           chrono.Q_from_AngX(chrono.CH_C_PI_2)))  
    system.AddLink(revolute)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Simple Pendulum')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, 0, 3))
    vis.AddTypicalLights()

    
    time_step = 0.001
    log_interval = 0.01  
    next_log_time = 0

    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)
        
        
        t = system.GetChTime()
        if t >= next_log_time:
            pos = pendulum.GetPos()
            vel = pendulum.GetPos_dt()
            print(f"Time: {t:.3f}s | Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) | "
                  f"Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
            next_log_time += log_interval

if __name__ == "__main__":
    main()