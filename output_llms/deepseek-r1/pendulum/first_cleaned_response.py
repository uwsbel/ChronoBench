import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
    
    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)  
    system.Add(ground)
    
    
    pendulum = chrono.ChBody()
    pendulum.SetMass(1.0)  
    pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.01, 0.1))  
    
    
    pendulum.SetPos(chrono.ChVectorD(0.5, -0.5, 0))
    
    
    box_shape = chrono.ChBoxShape()
    box_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.025, 0.5, 0.025)
    pendulum.AddVisualShape(box_shape, chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(chrono.CH_C_PI_2)))
    system.Add(pendulum)
    
    
    pivot = chrono.ChLinkLockRevolute()
    pivot.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
    system.Add(pivot)
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('PyChrono Pendulum')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1, 0.5, 2))
    vis.AddTypicalLights()
    
    
    time_step = 0.001
    step_number = 0
    log_interval = 20  
    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        system.DoStepDynamics(time_step)
        
        
        if step_number % log_interval == 0:
            pos = pendulum.GetPos()
            vel = pendulum.GetPos_dt()
            print(f"Time: {system.GetChTime():.3f}s | "
                  f"Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) | "
                  f"Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
        
        step_number += 1

if __name__ == "__main__":
    main()