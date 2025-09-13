import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    system.Add(ground)

    
    bar = chrono.ChBodyEasyBox(3.0, 0.2, 0.2, 1000)  
    bar.SetPos(chrono.ChVectorD(0, 0, 0))
    bar.SetRot(chrono.Q_from_AngY(chrono.CH_C_PI / 2))  
    system.Add(bar)

    
    sun_radius = 1.0
    sun = chrono.ChBodyEasyCylinder(sun_radius, 0.2, 1000)
    sun.SetPos(chrono.ChVectorD(0, 0, 0))
    sun.SetBodyFixed(True)  
    system.Add(sun)

    
    planet_radius = 0.5
    planet = chrono.ChBodyEasyCylinder(planet_radius, 0.2, 1000)
    planet.SetPos(chrono.ChVectorD(1.5, 0, 0))  
    system.Add(planet)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(bar, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    motor_speed = chrono.ChFunction_Const(0.5)  
    motor.SetSpeedFunction(motor_speed)
    system.Add(motor)

    
    planet_joint = chrono.ChLinkLockRevolute()
    planet_joint.Initialize(planet, bar, 
                            chrono.ChCoordsysD(chrono.ChVectorD(1.5, 0, 0)))
    system.Add(planet_joint)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Epicyclic Gear Simulation')
    vis.Initialize()
    vis.AddCamera(chrono.ChVectorD(0, -3, 2), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    
    sun_mat = chrono.ChVisualMaterial()
    sun_mat.SetDiffuseColor(chrono.ChColor(0.8, 0.2, 0.2))  
    sun.GetVisualShape(0).SetMaterial(0, sun_mat)

    bar_mat = chrono.ChVisualMaterial()
    bar_mat.SetDiffuseColor(chrono.ChColor(0.2, 0.6, 0.2))  
    bar.GetVisualShape(0).SetMaterial(0, bar_mat)

    planet_mat = chrono.ChVisualMaterial()
    planet_mat.SetDiffuseColor(chrono.ChColor(0.2, 0.2, 0.8))  
    planet.GetVisualShape(0).SetMaterial(0, planet_mat)

    
    time_step = 0.01
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()