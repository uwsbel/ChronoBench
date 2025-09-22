import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    chrono.SetChronoDataPath("/path/to/chrono/data/")

    
    system = chrono.ChSystemNSC()

    
    truss = chrono.ChBody()
    truss.SetBodyFixed(True)
    system.Add(truss)

    
    bar = chrono.ChBody()
    bar.SetMass(1.0)
    bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    bar.SetPos(chrono.ChVectorD(0, 0, 0))
    bar.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    system.Add(bar)

    
    gear1 = chrono.ChBody()
    gear1.SetMass(1.0)
    gear1.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    gear1.SetPos(chrono.ChVectorD(-2, 0, 0))
    system.Add(gear1)

    
    gear2 = chrono.ChBody()
    gear2.SetMass(1.0)
    gear2.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    gear2.SetPos(chrono.ChVectorD(2, 0, 0))
    system.Add(gear2)

    
    rev_joint1 = chrono.ChLinkRevolute()
    rev_joint1.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
    system.Add(rev_joint1)

    
    rev_joint2 = chrono.ChLinkRevolute()
    rev_joint2.Initialize(bar, gear1, chrono.ChCoordsysD(chrono.ChVectorD(-2, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
    system.Add(rev_joint2)

    
    rev_joint3 = chrono.ChLinkRevolute()
    rev_joint3.Initialize(truss, gear2, chrono.ChCoordsysD(chrono.ChVectorD(2, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
    system.Add(rev_joint3)

    
    gear_motor = chrono.ChLinkMotorRotationSpeed()
    gear_motor.Initialize(bar, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
    gear_motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI))  
    system.Add(gear_motor)

    
    gear_constraint = chrono.ChLinkGear()
    gear_constraint.Initialize(gear1, gear2, False, chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(0, 0, 1), -1)
    system.Add(gear_constraint)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Epicyclic Gear Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, 5, -5))
    vis.AddTypicalLights()

    
    gear_material = chrono.ChMaterialSurfaceNSC()
    gear_material.SetFriction(0.5)

    bar_mat = chrono.ChVisualMaterial()
    bar_mat.SetKdTexture(chrono.GetChronoDataPath() + 'blue.png')

    gear1_mat = chrono.ChVisualMaterial()
    gear1_mat.SetKdTexture(chrono.GetChronoDataPath() + 'red.png')

    gear2_mat = chrono.ChVisualMaterial()
    gear2_mat.SetKdTexture(chrono.GetChronoDataPath() + 'green.png')

    truss_shape = chrono.ChVisualShapeBox(1, 1, 1)
    truss_shape.SetMaterial(0, bar_mat)
    truss.AddVisualShape(truss_shape)

    bar_shape = chrono.ChVisualShapeBox(4, 0.2, 0.2)
    bar_shape.SetMaterial(0, bar_mat)
    bar.AddVisualShape(bar_shape)

    gear1_shape = chrono.ChVisualShapeCylinder(0.5, 0.2)
    gear1_shape.SetMaterial(0, gear1_mat)
    gear1.AddVisualShape(gear1_shape)

    gear2_shape = chrono.ChVisualShapeCylinder(0.5, 0.2)
    gear2_shape.SetMaterial(0, gear2_mat)
    gear2.AddVisualShape(gear2_shape)

    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(0.01)

if __name__ == "__main__":
    main()