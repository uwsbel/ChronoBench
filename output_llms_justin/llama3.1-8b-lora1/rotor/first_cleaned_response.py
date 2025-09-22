import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea
import math



def main():
    

    
    
    

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024,768)
    vis.SetWindowTitle('Jeffcott rotor with IGA beam')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddTypicalLights()

    
    

    
    
    

    
    ground = chrono.ChBodyEasyBox(10,1,1, 1000, True, False)
    ground.SetPos(chrono.ChVector3d(0,-.1,0))
    ground.SetFixed(True)
    sys.Add(ground)

    
    shaft = chrono.ChShaft(chrono.ChLinkShaft())
    shaft.SetSpindleAxis(chrono.ChVector3d(0,0,1))
    shaft.SetCollide(False)
    shaft.SetSpring(0)
    shaft.SetDashpot(0)
    shaft.SetTorsionalSpring(0)
    shaft.SetTorsionalDashpot(0)
    shaft.SetTorsionalConstant(1e-3)
    shaft.Initialize(sys)
    sys.Add(shaft)

    
    flywheel = chrono.ChBody()
    m = 1.0  
    I = 0.01  
    flywheel.SetMass(m)
    flywheel.SetInertiaXX(chrono.ChVector3d(I, I, I))
    flywheel.SetPos(chrono.ChVector3d(0,0,0))
    flywheel.SetRot(chrono.ChQuaterniond(1,0,0,0))
    flywheel.AddCollisionShape(chrono.ChCollisionShapeBox(m, 0.1, 0.1, 0.1))
    flywheel.SetFixed(False)
    flywheel.EnableCollision(True)
    sys.Add(flywheel)

    
    motor = chrono.ChLinkMotorRotationAngle()
    motor.SetSpindleAxis(chrono.ChVector3d(0,0,1))
    motor.SetAngle(0)
    motor.Initialize(flywheel, shaft.GetFirstBody())
    shaft.GetFirstBody().EnableCollision(False)
    sys.Add(motor)

    
    beam = fea.ChIGABeamFEA(shaft.GetFirstBody(), shaft.GetSecondBody(), 0.01, 0.1, 100, 100)
    beam.SetPlotType(fea.ChIGAPlotType_MESH)
    beam.SetCollisionType(fea.ChIGACollisionType_NONE)
    beam.SetMaterial(fea.ChIGAMaterial(2e11, 0, 0, 0, 0, 0))
    beam.Initialize(sys)
    sys.Add(beam)

    
    
    

    
    
    step_number = 0
    render_steps = 20

    
    
    render_steps_per_sec = render_steps / chrono.ChStepSizeToReal(1.0)

    
    step_size = 1e-3

    
    end_time = 5

    
    t1 = step_size
    t2 = 1
    t3 = 2
    t4 = 3
    t5 = end_time

    
    render = True
    render_once = True

    

    while (vis.Run()) :
        time = sys.GetChTime()

        if (render and time > t1 and time < t2 and not render_once):
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        elif (render and time > t2 and time < t3 and not render_once):
            vis.BeginScene()
            vis.Render()
            vis.LookAtCamera(chrono.ChVector3d(2,0,0),chrono.ChVector3d(0,0,0))
            vis.Render()
            vis.EndScene()
        elif (render and time > t3 and time < t4 and not render_once):
            vis.BeginScene()
            vis.Render()
            vis.LookAtCamera(chrono.ChVector3d(2,0,0),chrono.ChVector3d(0,0,0))
            vis.Render()
            vis.BeginScene()
            vis.Render()
            vis.LookAtCamera(chrono.ChVector3d(-2,0,0),chrono.ChVector3d(0,0,0))
            vis.Render()
            vis.EndScene()
        elif (render and time > t4 and time < t5 and not render_once):
            vis.BeginScene()
            vis.Render()
            vis.LookAtCamera(chrono.ChVector3d(-2,0,0),chrono.ChVector3d(0,0,0))
            vis.Render()
            vis.BeginScene()
            vis.Render()
            vis.LookAtCamera(chrono.ChVector3d(2,0,0),chrono.ChVector3d(0,0,0))
            vis.Render()
            vis.EndScene()

        if (render_once):
            render_once = False

        
        beam = fea.ChIGABodyFEAcast(beam)

        
        bodyA = shaft.GetFirstBody()
        bodyB = shaft.GetSecondBody()

        
        angA = bodyA.GetAngle()
        angB = bodyB.GetAngle()

        
        vis.BeginScene()
        vis.Render()
        vis.GetSceneManager().getActiveCamera().setTargetPoint(chrono.ChVector3d(0,0,0))
        vis.Render()
        vis.EndScene()

        
        motor_angle = 0.0
        if (time > 0.0):
            motor_angle = 2.0 * math.pi * 50.0 * time

        
        motor.SetAngle(motor_angle)

        
        shaft.SetAngleA(angA)
        shaft.SetAngleB(angB)

        
        flywheel.SetPos(chrono.ChVector3d(0,0,0))
        flywheel.SetRot(chrono.ChQuaterniond(1,0,0,0))

        
        shaft.GetFirstBody().EnableCollision(False)
        shaft.GetSecondBody().EnableCollision(False)

        
        beam.GetBody().EnableCollision(False)

        
        flywheel.EnableCollision(False)

        
        motor.GetFirstBody().EnableCollision(False)
        motor.GetSecondBody().EnableCollision(False)

        
        ground.EnableCollision(False)

        
        
        
        
        
        
        

        
        sys.DoStepDynamics(step_size)

        
        if (time > t5):
            render = False

        
        sys.DoStepDynamics(step_size)

        
        step_number += 1

        
        if (step_number % render_steps == 0):
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        
        if (time >= end_time):
            break

    return





sys = chrono.ChSystemNSC()



def my_random(min, max):
    return min + (max - min) * chrono.ChRandom().GetReal()



material = chrono.ChContactMaterialNSC()
material.SetFriction(0.5)
material.SetDampingF(0.002)


ground_mat = material.GetMaterial0()
patch = chrono.ChCollisionShapeBox(material, chrono.ChVector3d(.5,.5,.5))
ground.AddCollisionShape(patch)







fea_sys = chrono.ChSystemNSC()
fea_sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)








vis = chrono.IrrlichtVisualizer()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('IGA Beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddTypicalLights()







beam = fea.ChIGABeamFEA(0, 0, 0, 0, 0, 0, 0.01, 0.1, 100, 100)
beam.SetPlotType(fea.ChIGAPlotType_MESH)
beam.SetCollisionType(fea.ChIGACollisionType_NONE)
beam.SetMaterial(fea.ChIGAMaterial(2e11, 0, 0, 0, 0, 0))
beam.Initialize(fea_sys)







shaftA = chrono.ChShaft(chrono.ChLinkShaft())
shaftA.SetSpindleAxis(chrono.ChVector3d(0,0,1))
shaftA.SetCollide(False)
shaftA.SetSpring(0)
shaftA.SetDashpot(0)
shaftA.SetTorsionalSpring(0)
shaftA.SetTorsionalDashpot(0)
shaftA.SetTorsionalConstant(1e-3)
shaftA.Initialize(fea_sys)
fea_sys.Add(shaftA)

shaftB = chrono.ChShaft(chrono.ChLinkShaft())
shaftB.SetSpindleAxis(chrono.ChVector3d(0,0,1))
shaftB.SetCollide(False)
shaftB.SetSpring(0)
shaftB.SetDashpot(0)
shaftB.SetTorsionalSpring(0)
shaftB.SetTorsionalDashpot(0)
shaftB.SetTorsionalConstant(1e-3)
shaftB.Initialize(fea_sys)
fea_sys.Add(shaftB)







flywheel = chrono.ChBody()
m = 1.0  
I = 0.01  
flywheel.SetMass(m)
flywheel.SetInertiaXX(chrono.ChVector3d(I, I, I))
flywheel.SetPos(chrono.ChVector3d(0,0,0))
flywheel.SetRot(chrono.ChQuaterniond(1,0,0,0))
flywheel.AddCollisionShape(chrono.ChCollisionShapeBox(m, 0.1, 0.1, 0.1))
flywheel.SetFixed(False)
flywheel.EnableCollision(True)
fea_sys.Add(flywheel)







motor = chrono.ChLinkMotorRotationAngle()
motor.SetSpindleAxis(chrono.ChVector3d(0,0,1))
motor.SetAngle(0)
motor.Initialize(flywheel, shaftA.GetFirstBody())
fea_sys.Add(motor)







ground = chrono.ChBodyEasyBox(10,1,1, 1000, True, False)
ground.SetPos(chrono.ChVector3d(0,-.1,0))
ground.SetFixed(True)
fea_sys.Add(ground)








render_steps = 20


step_size = 1e-3


end_time = 5


t1 = step_size
t2 = 1
t3 = 2
t4 = 3
t5 = end_time


render = True
render_once = True

while (vis.Run()) :
    time = fea_sys.GetChTime()

    if (render and time > t1 and time < t2 and not render_once):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    elif (render and time > t2 and time < t3 and not render_once):
        vis.BeginScene()
        vis.Render()
        vis.LookAtCamera(chrono.ChVector3d(2,0,0),chrono.ChVector3d(0,0,0))
        vis.Render()
        vis.EndScene()
    elif (render and time > t3 and time < t4 and not render_once):
        vis.BeginScene()
        vis.Render()
        vis.LookAtCamera(chrono.ChVector3d(2,0,0),chrono.ChVector3d(0,0,0))
        vis.Render()
        vis.BeginScene()
        vis.Render()
        vis.LookAtCamera(chrono.ChVector3d(-2,0,0),chrono.ChVector3d(0,0,0))
        vis.Render()
        vis.EndScene()
    elif (render and time > t4 and time < t5 and not render_once):
        vis.BeginScene()
        vis.Render()
        vis.LookAtCamera(chrono.ChVector3d(-2,0,0),chrono.ChVector3d(0,0,0))
        vis.Render()
        vis.BeginScene()
        vis.Render()
        vis.LookAtCamera(chrono.ChVector3d(2,0,0),chrono.ChVector3d(0,0,0))
        vis.Render()
        vis.EndScene()

    if (render_once):
        render_once = False

    
    beam = fea.ChIGABodyFEAcast(beam)

    
    bodyA = shaftA.GetFirstBody()
    bodyB = shaftB.GetSecondBody()

    
    angA = bodyA.GetAngle()
    angB = bodyB.GetAngle()

    
    vis.BeginScene()
    vis.Render()
    vis.GetSceneManager().getActiveCamera().setTargetPoint(chrono.ChVector3d(0,0,0))
    vis.Render()
    vis.EndScene()

    
    motor_angle = 0.0
    if (time > 0.0):
        motor_angle = 2.0 * math.pi * 50.0 * time

    
    motor.SetAngle(motor_angle)

    
    shaftA.SetAngleA(angA)
    shaftB.SetAngleB(angB)

    
    flywheel.SetPos(chrono.ChVector3d(0,0,0))
    flywheel.SetRot(chrono.ChQuaterniond(1,0,0,0))

    
    shaftA.GetFirstBody().EnableCollision(False)
    shaftB.GetSecondBody().EnableCollision(False)

    
    beam.GetBody().EnableCollision(False)

    
    flywheel.EnableCollision(False)

    
    motor.GetFirstBody().EnableCollision(False)
    motor.GetSecondBody().EnableCollision(False)

    
    ground.EnableCollision(False)

    
    
    
    
    
    
    

    
    fea_sys.DoStepDynamics(step_size)

    
    if (time > t5):
        render = False

    
    fea_sys.DoStepDynamics(step_size)

    
    step_number += 1

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    if (time >= end_time):
        break

main()