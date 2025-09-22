import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea
import math

# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # --------------------------
    # Create the various modules
    # --------------------------

    # Create the Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024,768)
    vis.SetWindowTitle('Jeffcott rotor with IGA beam')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddTypicalLights()

    # Create the FEA solver
    #fea.SetChChronoCollisionType(chrono.ChContactMaterialSMC.GetDefaultMaterial().GetCollisionType())

    # --------------------------
    # Create the mechanical system
    # --------------------------

    # Create the ground body
    ground = chrono.ChBodyEasyBox(10,1,1, 1000, True, False)
    ground.SetPos(chrono.ChVector3d(0,-.1,0))
    ground.SetFixed(True)
    sys.Add(ground)

    # Create the shaft
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

    # Create the flywheel
    flywheel = chrono.ChBody()
    m = 1.0  # mass
    I = 0.01  # moment of inertia
    flywheel.SetMass(m)
    flywheel.SetInertiaXX(chrono.ChVector3d(I, I, I))
    flywheel.SetPos(chrono.ChVector3d(0,0,0))
    flywheel.SetRot(chrono.ChQuaterniond(1,0,0,0))
    flywheel.AddCollisionShape(chrono.ChCollisionShapeBox(m, 0.1, 0.1, 0.1))
    flywheel.SetFixed(False)
    flywheel.EnableCollision(True)
    sys.Add(flywheel)

    # Create the motor
    motor = chrono.ChLinkMotorRotationAngle()
    motor.SetSpindleAxis(chrono.ChVector3d(0,0,1))
    motor.SetAngle(0)
    motor.Initialize(flywheel, shaft.GetFirstBody())
    shaft.GetFirstBody().EnableCollision(False)
    sys.Add(motor)

    # Create the beam
    beam = fea.ChIGABeamFEA(shaft.GetFirstBody(), shaft.GetSecondBody(), 0.01, 0.1, 100, 100)
    beam.SetPlotType(fea.ChIGAPlotType_MESH)
    beam.SetCollisionType(fea.ChIGACollisionType_NONE)
    beam.SetMaterial(fea.ChIGAMaterial(2e11, 0, 0, 0, 0, 0))
    beam.Initialize(sys)
    sys.Add(beam)

    # --------------------------
    # Simulation loop
    # --------------------------

    # Number of steps per beam time
    # (for real-time simulation, we want to render at 50 Hz)
    step_number = 0
    render_steps = 20

    # Initialize simulation frame rates
    # (render steps per second)
    render_steps_per_sec = render_steps / chrono.ChStepSizeToReal(1.0)

    # Simulation step size
    step_size = 1e-3

    # Simulation end time
    end_time = 5

    # Number of steps
    t1 = step_size
    t2 = 1
    t3 = 2
    t4 = 3
    t5 = end_time

    # Render automatically adjust
    render = True
    render_once = True

    # --------------------------

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

        # Access the FEA beam
        beam = fea.ChIGABodyFEAcast(beam)

        # Access the shaft bodies
        bodyA = shaft.GetFirstBody()
        bodyB = shaft.GetSecondBody()

        # Get the shaft angles
        angA = bodyA.GetAngle()
        angB = bodyB.GetAngle()

        # Update the visualization
        vis.BeginScene()
        vis.Render()
        vis.GetSceneManager().getActiveCamera().setTargetPoint(chrono.ChVector3d(0,0,0))
        vis.Render()
        vis.EndScene()

        # Update the motor angle
        motor_angle = 0.0
        if (time > 0.0):
            motor_angle = 2.0 * math.pi * 50.0 * time

        # Update the motor
        motor.SetAngle(motor_angle)

        # Update the shafts
        shaft.SetAngleA(angA)
        shaft.SetAngleB(angB)

        # Update the flywheel
        flywheel.SetPos(chrono.ChVector3d(0,0,0))
        flywheel.SetRot(chrono.ChQuaterniond(1,0,0,0))

        # Update the shaft visualization
        shaft.GetFirstBody().EnableCollision(False)
        shaft.GetSecondBody().EnableCollision(False)

        # Update the beam visualization
        beam.GetBody().EnableCollision(False)

        # Update the flywheel visualization
        flywheel.EnableCollision(False)

        # Update the motor visualization
        motor.GetFirstBody().EnableCollision(False)
        motor.GetSecondBody().EnableCollision(False)

        # Update the ground body
        ground.EnableCollision(False)

        # Enable collision shapes
        #shaft.GetFirstBody().GetCollisionModel().SetAllShapesToCollide(True)
        #shaft.GetSecondBody().GetCollisionModel().SetAllShapesToCollide(True)
        #beam.GetBody().GetCollisionModel().SetAllShapesToCollide(True)
        #flywheel.GetCollisionModel().SetAllShapesToCollide(True)
        #motor.GetFirstBody().GetCollisionModel().SetAllShapesToCollide(True)
        #motor.GetSecondBody().GetCollisionModel().SetAllShapesToCollide(True)

        # Update the collision system
        sys.DoStepDynamics(step_size)

        # Render automatically adjust
        if (time > t5):
            render = False

        # Advance simulation for one timestep
        sys.DoStepDynamics(step_size)

        # Number of steps
        step_number += 1

        # Spin for a bit before changing the camera view
        if (step_number % render_steps == 0):
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # End simulation loop
        if (time >= end_time):
            break

    return

# -----------------
# Create the system
# -----------------

sys = chrono.ChSystemNSC()

# This is an example of a function that generates a random number
# between min and max
def my_random(min, max):
    return min + (max - min) * chrono.ChRandom().GetReal()

# The collision material to be used by all shapes in the simulation.
# This is a very basic material that only considers friction.
material = chrono.ChContactMaterialNSC()
material.SetFriction(0.5)
material.SetDampingF(0.002)

# Create a collision shape for the ground, set its material, and add it to the ground body.
ground_mat = material.GetMaterial0()
patch = chrono.ChCollisionShapeBox(material, chrono.ChVector3d(.5,.5,.5))
ground.AddCollisionShape(patch)

# -----------------
#  Create the FEA
#  system
# -----------------

# Create the FEA system
fea_sys = chrono.ChSystemNSC()
fea_sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# -----------------
#  Create the
#  Irrlicht
#  application
# -----------------

# Create the Irrlicht application
vis = chrono.IrrlichtVisualizer()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('IGA Beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddTypicalLights()

# -----------------
#  Create the
#  IGA beam
# -----------------

# Create the IGA beam
beam = fea.ChIGABeamFEA(0, 0, 0, 0, 0, 0, 0.01, 0.1, 100, 100)
beam.SetPlotType(fea.ChIGAPlotType_MESH)
beam.SetCollisionType(fea.ChIGACollisionType_NONE)
beam.SetMaterial(fea.ChIGAMaterial(2e11, 0, 0, 0, 0, 0))
beam.Initialize(fea_sys)

# -----------------
#  Create the
#  shafts
# -----------------

# Create the shafts
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

# -----------------
#  Create the
#  flywheel
# -----------------

# Create the flywheel
flywheel = chrono.ChBody()
m = 1.0  # mass
I = 0.01  # moment of inertia
flywheel.SetMass(m)
flywheel.SetInertiaXX(chrono.ChVector3d(I, I, I))
flywheel.SetPos(chrono.ChVector3d(0,0,0))
flywheel.SetRot(chrono.ChQuaterniond(1,0,0,0))
flywheel.AddCollisionShape(chrono.ChCollisionShapeBox(m, 0.1, 0.1, 0.1))
flywheel.SetFixed(False)
flywheel.EnableCollision(True)
fea_sys.Add(flywheel)

# -----------------
#  Create the
#  motor
# -----------------

# Create the motor
motor = chrono.ChLinkMotorRotationAngle()
motor.SetSpindleAxis(chrono.ChVector3d(0,0,1))
motor.SetAngle(0)
motor.Initialize(flywheel, shaftA.GetFirstBody())
fea_sys.Add(motor)

# -----------------
#  Create the
#  ground
# -----------------

# Create the ground body
ground = chrono.ChBodyEasyBox(10,1,1, 1000, True, False)
ground.SetPos(chrono.ChVector3d(0,-.1,0))
ground.SetFixed(True)
fea_sys.Add(ground)

# -----------------
#  Simulation
#  loop
# -----------------

# Number of steps per beam time
# (for real-time simulation, we want to render at 50 Hz)
render_steps = 20

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 5

# Number of steps
t1 = step_size
t2 = 1
t3 = 2
t4 = 3
t5 = end_time

# Render automatically adjust
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

    # Access the FEA beam
    beam = fea.ChIGABodyFEAcast(beam)

    # Access the shaft bodies
    bodyA = shaftA.GetFirstBody()
    bodyB = shaftB.GetSecondBody()

    # Get the shaft angles
    angA = bodyA.GetAngle()
    angB = bodyB.GetAngle()

    # Update the visualization
    vis.BeginScene()
    vis.Render()
    vis.GetSceneManager().getActiveCamera().setTargetPoint(chrono.ChVector3d(0,0,0))
    vis.Render()
    vis.EndScene()

    # Update the motor angle
    motor_angle = 0.0
    if (time > 0.0):
        motor_angle = 2.0 * math.pi * 50.0 * time

    # Update the motor
    motor.SetAngle(motor_angle)

    # Update the shafts
    shaftA.SetAngleA(angA)
    shaftB.SetAngleB(angB)

    # Update the flywheel
    flywheel.SetPos(chrono.ChVector3d(0,0,0))
    flywheel.SetRot(chrono.ChQuaterniond(1,0,0,0))

    # Update the shaft visualization
    shaftA.GetFirstBody().EnableCollision(False)
    shaftB.GetSecondBody().EnableCollision(False)

    # Update the beam visualization
    beam.GetBody().EnableCollision(False)

    # Update the flywheel visualization
    flywheel.EnableCollision(False)

    # Update the motor visualization
    motor.GetFirstBody().EnableCollision(False)
    motor.GetSecondBody().EnableCollision(False)

    # Update the ground body
    ground.EnableCollision(False)

    # Enable collision shapes
    #shaftA.GetFirstBody().GetCollisionModel().SetAllShapesToCollide(True)
    #shaftB.GetSecondBody().GetCollisionModel().SetAllShapesToCollide(True)
    #beam.GetBody().GetCollisionModel().SetAllShapesToCollide(True)
    #flywheel.GetCollisionModel().SetAllShapesToCollide(True)
    #motor.GetFirstBody().GetCollisionModel().SetAllShapesToCollide(True)
    #motor.GetSecondBody().GetCollisionModel().SetAllShapesToCollide(True)

    # Update the collision system
    fea_sys.DoStepDynamics(step_size)

    # Render automatically adjust
    if (time > t5):
        render = False

    # Advance simulation for one timestep
    fea_sys.DoStepDynamics(step_size)

    # Number of steps
    step_number += 1

    # Spin for a bit before changing the camera view
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # End simulation loop
    if (time >= end_time):
        break

main()