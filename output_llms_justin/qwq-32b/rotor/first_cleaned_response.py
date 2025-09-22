import pychrono as chrono
import pychrono.igabase as igabase
import pychrono.postprocess as postprocess
import pychrono.irrlicht as irr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
    ground = chrono.ChBody()
    ground.SetIdentifier(-1)
    ground.SetBodyFixed(True)
    ground.SetCollide(False)
    system.Add(ground)

    
    control_points = [
        chrono.ChVectorD(0, 0, 0),
        chrono.ChVectorD(0.5, 0, 0),
        chrono.ChVectorD(1, 0, 0)
    ]
    knots = [0, 0, 0, 1, 1, 1]
    weights = [1, 1, 1]

    curve = chrono.ChNurbsCurve()
    curve.SetDegree(2)
    curve.SetControlPoints(control_points)
    curve.SetKnots(knots)
    curve.SetWeights(weights)

    
    section = chrono.ChBeamSectionIga()
    section.SetMaterialYoungModulus(2.1e11)  
    section.SetMaterialPoissonRatio(0.3)
    section.SetArea(0.01)  
    section.SetIy(0.0001)  
    section.SetIz(0.0001)  
    section.SetJ(0.0002)   

    
    beam = chrono.ChBeamIga()
    beam.Initialize(curve, section, 1)  
    system.Add(beam)

    
    rot = chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(1, 0, 0))
    coordsys = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), rot)
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(beam, ground, coordsys)
    motor.SetMotorMode(chrono.ChLinkMotorRotationSpeed.MOTOR_MODE_SPEED)
    motor.SetSpeed(chrono.ChFunction.ConstantFunction(10))  
    system.Add(motor)

    
    flywheel = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
    flywheel.SetPos(chrono.ChVectorD(0.5, 0, 0))
    system.Add(flywheel)

    
    joint_fly = chrono.ChLinkLockPointFrame()
    joint_fly.Initialize(flywheel, beam, chrono.ChVectorD(0.5, 0, 0))
    system.Add(joint_fly)

    
    application = irr.ChIrrApp(system, 'Jeffcott Rotor Simulation', irr.dimension2du(800, 600))
    application.AddTypicalLights()
    application.AddTypicalCamera(chrono.ChVectorD(0.5, 1, 0.5), chrono.ChVectorD(0.5, 0, 0))
    application.AddAllTypicalEventHandlers()

    
    post = postprocess.ChPostProcess()
    post.AddFEMMesh(beam)
    application.SetPostProcess(post)

    
    application.AssetBindAll()
    application.AssetUpdateAll()

    
    while application.GetDevice().run():
        system.DoStepDynamics(1e-3)
        application.BeginScene()
        application.DrawAll()
        application.EndScene()

if __name__ == '__main__':
    main()