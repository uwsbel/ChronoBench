import chrono as ch
import chrono.irrlicht as irr


ch.CHGlobals.Initialize()


app = irr.IrrlichtApplication()


system = ch.ChSystem()


ground = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False)
ground.SetPos(ch.ChVector3d(0, -5, 0))
ground.SetFixed(True)
system.AddBody(ground)


pendulum = ch.ChBodyEasyBox(0.1, 1, 0.1, 10, False, False)
pendulum.SetPos(ch.ChVector3d(0, 0, 0))
pendulum.SetRot(ch.ChQuaternion(0, 0, 0, 1))
system.AddBody(pendulum)


joint = ch.ChRevoluteJoint(ground, pendulum, ch.ChVector3d(0, -5, 0), ch.ChVector3d(0, 0, 0))
joint.SetModulusOfElasticity(1e6)
joint.SetDissipation(0.5)
system.AddJoint(joint)


torque = ch.ChTorque()
torque.SetTorque(0, 0, -0.5)
pendulum.AddTorque(torque)


visual_system = irr.IrrlichtSystem(system)
visual_system.SetWindowSize(ch.ChVector2i(800, 600))
visual_system.SetWindowTitle("Simple Pendulum Simulation")
visual_system.EnableShadows()
app.SetVisualSystem(visual_system)


while True:
    
    system.DoStepDynamics(0.01)

    
    print("Pendulum position: ", pendulum.GetPos())
    print("Pendulum velocity: ", pendulum.GetPos_dt())

    
    app.DoStep()