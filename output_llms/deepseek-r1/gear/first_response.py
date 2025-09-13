import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    # 1. Initialize system and set gravity
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Zero gravity

    # 2. Create a fixed truss (ground)
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    system.Add(ground)

    # 3. Create rotating bar (carrier)
    bar = chrono.ChBodyEasyBox(3.0, 0.2, 0.2, 1000)  # Dimensions: 3m x 0.2m x 0.2m
    bar.SetPos(chrono.ChVectorD(0, 0, 0))
    bar.SetRot(chrono.Q_from_AngY(chrono.CH_C_PI / 2))  # Rotate to align with X-axis
    system.Add(bar)

    # 4. Create sun gear (fixed to ground)
    sun_radius = 1.0
    sun = chrono.ChBodyEasyCylinder(sun_radius, 0.2, 1000)
    sun.SetPos(chrono.ChVectorD(0, 0, 0))
    sun.SetBodyFixed(True)  # Fix to ground
    system.Add(sun)

    # 5. Create planet gear
    planet_radius = 0.5
    planet = chrono.ChBodyEasyCylinder(planet_radius, 0.2, 1000)
    planet.SetPos(chrono.ChVectorD(1.5, 0, 0))  # Positioned at end of bar
    system.Add(planet)

    # 6. Add bar-ground revolute joint with motor
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(bar, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    motor_speed = chrono.ChFunction_Const(0.5)  # 0.5 rad/sec constant speed
    motor.SetSpeedFunction(motor_speed)
    system.Add(motor)

    # 7. Add planet-bar revolute joint
    planet_joint = chrono.ChLinkLockRevolute()
    planet_joint.Initialize(planet, bar, 
                            chrono.ChCoordsysD(chrono.ChVectorD(1.5, 0, 0)))
    system.Add(planet_joint)

    # 8. Set up visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Epicyclic Gear Simulation')
    vis.Initialize()
    vis.AddCamera(chrono.ChVectorD(0, -3, 2), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    # 9. Assign materials for visualization
    sun_mat = chrono.ChVisualMaterial()
    sun_mat.SetDiffuseColor(chrono.ChColor(0.8, 0.2, 0.2))  # Red
    sun.GetVisualShape(0).SetMaterial(0, sun_mat)

    bar_mat = chrono.ChVisualMaterial()
    bar_mat.SetDiffuseColor(chrono.ChColor(0.2, 0.6, 0.2))  # Green
    bar.GetVisualShape(0).SetMaterial(0, bar_mat)

    planet_mat = chrono.ChVisualMaterial()
    planet_mat.SetDiffuseColor(chrono.ChColor(0.2, 0.2, 0.8))  # Blue
    planet.GetVisualShape(0).SetMaterial(0, planet_mat)

    # 10. Simulation loop
    time_step = 0.01
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()