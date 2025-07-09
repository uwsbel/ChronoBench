import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

def main():
    
    sys = chrono.ChSystemNSC()

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    sys.Add(ground)

    
    pendulum = chrono.ChBody()
    pendulum.SetMass(1.0)  
    pendulum.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))  
    pendulum.SetPos(chrono.ChVector3d(0, -1, 0))  
    sys.Add(pendulum)

    
    rev_joint = chrono.ChLinkLockRevolute()
    rev_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0)))
    sys.Add(rev_joint)

    
    ground_shape = chrono.ChVisualShapeBox(1, 0.1, 1)
    ground_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    ground.AddVisualShape(ground_shape)

    pendulum_shape = chrono.ChVisualShapeBox(0.1, 2, 0.1)
    pendulum_shape.SetColor(chrono.ChColor(1, 0, 0))
    pendulum.AddVisualShape(pendulum_shape, chrono.ChFrameD(chrono.ChVector3d(0, -1, 0)))

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(800, 600)
    vis.SetWindowTitle('Simple Pendulum Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3, 3, -3))
    vis.AddTypicalLights()

    
    time_step = 0.01
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        sys.DoStepDynamics(time_step)
        vis.EndScene()

        
        if sys.GetChTime() % 1 < time_step:
            pos = pendulum.GetPos()
            vel = pendulum.GetPos_dt()
            print(f"Time: {sys.GetChTime():.2f}, Position: {pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}, Velocity: {vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f}")

if __name__ == "__main__":
    main()