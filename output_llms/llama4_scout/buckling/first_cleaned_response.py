import chrono as ch


ch.ChEngine.Initialize()


system = ch.ChSystemNSC()


system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))


system.SetSolverType(ch.ChSolverType_PEND3)
system.SetTimestepperType(ch.ChTimestepperType_EULER_IMPLICIT_LINEARIZED)


beam_mesh = ch.ChMesh()


beam_length = 10.0
beam_width = 0.1
beam_height = 0.1
num_elements = 10


for i in range(num_elements):
    node0 = ch.ChNodeFEA(beam_mesh.GetNumNodes())
    node1 = ch.ChNodeFEA(beam_mesh.GetNumNodes() + 1)
    
    
    node0.SetPos(ch.ChVector3d(i * beam_length / num_elements, 0, 0))
    node1.SetPos(ch.ChVector3d((i + 1) * beam_length / num_elements, 0, 0))
    
    
    elem = ch.ChElementBeamTimoshenko()
    elem.SetNodes(node0, node1)
    elem.SetSection(ch.ChSectionBeamSimple(beam_width, beam_height, 0.1 * beam_width * beam_height * beam_height / 12))
    
    
    beam_mesh.AddElement(elem)


beam_mesh.AddNode(node0)
beam_mesh.AddNode(node1)


system.Add(beam_mesh)


class CustomMotorFunction(ch.ChFunction):
    def __init__(self):
        super(CustomMotorFunction, self).__init__()
    
    def Get_y(self, x):
        return 1.0 * ch.CH_C_PI * x


motor = ch.ChLinkMotor()
motor.SetMotorFunction(CustomMotorFunction())


system.Add(motor)


beam_mesh.GetNode(0).SetFixed(True)
beam_mesh.GetNode(num_elements).SetFixed(False)


force = ch.ChForce()
force.SetLink(beam_mesh.GetNode(num_elements // 2))
force.SetForce(ch.ChVector3d(0, -1000, 0))
system.Add(force)


constraint = ch.ChLinkLock()
constraint.SetLinkA(motor)
constraint.SetLinkB(beam_mesh.GetNode(num_elements // 2))
system.Add(constraint)


app = ch.ChIrrApp(system, "Beam Buckling Simulation")


app.SetCameraPosition(ch.ChVector3d(0, 0, 10))


app.Run()

import chrono as ch
import chrono.irrlicht as irr

def main():
    
    ch.ChEngine.Initialize()

    
    system = ch.ChSystemNSC()

    
    system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))

    
    system.SetSolverType(ch.ChSolverType_PEND3)
    system.SetTimestepperType(ch.ChTimestepperType_EULER_IMPLICIT_LINEARIZED)

    
    beam_mesh = ch.ChMesh()

    
    beam_length = 10.0
    beam_width = 0.1
    beam_height = 0.1
    num_elements = 10

    
    for i in range(num_elements):
        node0 = ch.ChNodeFEA(beam_mesh.GetNumNodes())
        node1 = ch.ChNodeFEA(beam_mesh.GetNumNodes() + 1)
        
        
        node0.SetPos(ch.ChVector3d(i * beam_length / num_elements, 0, 0))
        node1.SetPos(ch.ChVector3d((i + 1) * beam_length / num_elements, 0, 0))
        
        
        elem = ch.ChElementBeamTimoshenko()
        elem.SetNodes(node0, node1)
        elem.SetSection(ch.ChSectionBeamSimple(beam_width, beam_height, 0.1 * beam_width * beam_height * beam_height / 12))
        
        
        beam_mesh.AddElement(elem)

    
    beam_mesh.AddNode(node0)
    beam_mesh.AddNode(node1)

    
    system.Add(beam_mesh)

    
    class CustomMotorFunction(ch.ChFunction):
        def __init__(self):
            super(CustomMotorFunction, self).__init__()
        
        def Get_y(self, x):
            return 1.0 * ch.CH_C_PI * x

    
    motor = ch.ChLinkMotor()
    motor.SetMotorFunction(CustomMotorFunction())

    
    system.Add(motor)

    
    beam_mesh.GetNode(0).SetFixed(True)
    beam_mesh.GetNode(num_elements).SetFixed(False)

    
    force = ch.ChForce()
    force.SetLink(beam_mesh.GetNode(num_elements // 2))
    force.SetForce(ch.ChVector3d(0, -1000, 0))
    system.Add(force)

    
    constraint = ch.ChLinkLock()
    constraint.SetLinkA(motor)
    constraint.SetLinkB(beam_mesh.GetNode(num_elements // 2))
    system.Add(constraint)

    
    app = irr.ChIrrApp(system, "Beam Buckling Simulation")

    
    app.SetCameraPosition(ch.ChVector3d(0, 0, 10))

    
    app.Run()

if __name__ == "__main__":
    main()