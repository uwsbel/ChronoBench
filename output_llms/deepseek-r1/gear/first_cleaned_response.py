import pychrono as chrono
import pychrono.irrlicht as irr

def main():
    
    system = chrono.ChSystemNSC()
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

    
    red_material = chrono.ChVisualMaterial()
    red_material.SetDiffuseColor(chrono.ChColor(0.8, 0.2, 0.2))
    blue_material = chrono.ChVisualMaterial()
    blue_material.SetDiffuseColor(chrono.ChColor(0.2, 0.2, 0.8))
    green_material = chrono.ChVisualMaterial()
    green_material.SetDiffuseColor(chrono.ChColor(0.2, 0.8, 0.2))

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    system.Add(ground)

    
    sun_radius = 0.5
    sun = chrono.ChBody()
    sun.SetPos(chrono.ChVectorD(0, 0, 0))
    sun_shape = chrono.ChCylinderShape()
    sun_shape.GetCylinderGeometry().rad = sun_radius
    sun_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -0.2)
    sun_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 0.2)
    sun.AddVisualShape(sun_shape)
    sun.GetVisualShape(0).SetMaterial(0, red_material)
    system.Add(sun)

    
    sun_to_ground = chrono.ChLinkLockRevolute()
    sun_to_ground.Initialize(sun, ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
    system.AddLink(sun_to_ground)

    
    arm_length = sun_radius * 1.8
    arm = chrono.ChBody()
    arm.SetPos(chrono.ChVectorD(0, 0, 0))
    arm_shape = chrono.ChBoxShape()
    arm_shape.GetBoxGeometry().Size = chrono.ChVectorD(arm_length/2, 0.05, 0.05)
    arm.AddVisualShape(arm_shape, chrono.ChFrameD(chrono.ChVectorD(arm_length/2, 0, 0)))
    arm.GetVisualShape(0).SetMaterial(0, green_material)
    system.Add(arm)

    
    motor = chrono.ChLinkMotorRotationAngle()
    motor.Initialize(arm, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    motor_func = chrono.ChFunction_Ramp(0, chrono.CH_C_PI / 4)  
    motor.SetAngleFunction(motor_func)
    system.AddLink(motor)

    
    planet_radius = sun_radius * 0.4
    planet = chrono.ChBody()
    planet.SetPos(chrono.ChVectorD(arm_length, 0, 0))
    planet_shape = chrono.ChCylinderShape()
    planet_shape.GetCylinderGeometry().rad = planet_radius
    planet_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -0.2)
    planet_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 0.2)
    planet.AddVisualShape(planet_shape)
    planet.GetVisualShape(0).SetMaterial(0, blue_material)
    system.Add(planet)

    
    planet_to_arm = chrono.ChLinkLockRevolute()
    planet_to_arm.Initialize(planet, arm, 
                            chrono.ChCoordsysD(chrono.ChVectorD(arm_length, 0, 0)))
    system.AddLink(planet_to_arm)

    
    gear_train = chrono.ChLinkGear()
    gear_train.Initialize(sun, planet, ground, 
                         chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)),
                         chrono.ChFrameD(chrono.ChVectorD(arm_length, 0, 0)))
    ratio = planet_radius / sun_radius  
    gear_train.SetTransmissionRatio(-ratio)
    system.AddLink(gear_train)

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Epicyclic Gear System')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(2, 1.5, 1))
    vis.AddTypicalLights()

    
    time_step = 0.01
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == '__main__':
    main()