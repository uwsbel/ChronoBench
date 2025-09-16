import pychrono as chrono
import pychrono.irrlicht as chronoirrl

def main():
    
    my_system = chrono.ChSystemNSC()

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    
    ground_shape = chrono.ChSphereShape()
    ground_shape.GetSphereGeometry().rad = 0.05
    ground.AddAsset(ground_shape)
    ground.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.5, 0.5, 0.5)))  
    my_system.Add(ground)

    
    pendulum_radius = 0.1  
    pendulum_mass = 1.0    
    
    density = pendulum_mass / ( (4/3)*chrono.CH_C_PI*(pendulum_radius**3) )
    pendulum = chrono.ChBodyEasySphere(pendulum_radius, density, True, True)
    pendulum.SetPos(chrono.ChVectorD(0.5, 0, -chrono.sqrt(3)/2))  
    pendulum.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.8, 0, 0)))  
    my_system.Add(pendulum)

    
    revolute = chrono.ChLinkLockRevolute()
    revolute.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
    
    revolute.SetAxis(chrono.ChVectorD(0, 1, 0))
    my_system.AddLink(revolute)

    
    vis = chronoirrl.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Simple Pendulum Simulation')
    vis.SetAntialiasing(0)
    vis.SetCameraPosition(chrono.ChVectorD(0, 3, 3))
    vis.SetCameraRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI/4, chrono.ChVectorD(1,0,0)))
    my_system.SetVisualSystem(vis)

    
    my_system.SetupSimulation()
    my_system.DoStepDynamics(0)  

    
    vis.BEGIN()

    
    step = 0
    while vis.Run() and my_system.GetChTime() < 10:
        my_system.DoStepDynamics(0.005)  

        
        if step % 10 == 0:
            pos = pendulum.GetPos()
            vel = pendulum.GetPos_dt()  
            print(f"Time: {my_system.GetChTime():.2f}s | Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) | Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")

        vis.Render()
        step += 1

    vis.END()

if __name__ == '__main__':
    main()