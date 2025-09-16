import chrono as ch
import numpy as np


ch.ChEngine.Initialize()


system = ch.ChSystemNSC()


contact_material = ch.ChContactMaterial()
contact_material.SetFrictionCoefficient(0.5)
contact_material.SetRestitutionCoefficient(0.5)


beam_length = 10.0
beam_width = 0.1
beam_height = 0.1
num_elements = 10

beam_fea = ch.ChBeamSectionEuler()
beam_fea.SetLength(beam_length / num_elements)
beam_fea.SetWidth(beam_width)
beam_fea.SetThickness(beam_height)
beam_fea.SetMaterial(ch.ChMaterialBeamSectionEuler(200e9, 0.3))

beam_body = ch.ChBody()
beam_body.SetPos([0, 0, 0])
system.AddBody(beam_body)

for i in range(num_elements):
    beam_element = ch.ChElementBeamEuler(beam_fea, beam_body, beam_body, 
                                        [beam_length / num_elements * i, 0, 0], 
                                        [beam_length / num_elements * (i + 1), 0, 0])
    system.AddElement(beam_element)


class CustomMotorFunction(ch.ChFunction):
    def __init__(self):
        super(CustomMotorFunction, self).__init__()
        self.rot_angle = 0

    def Get_y(self, t):
        self.rot_angle += 0.1 * ch.CH_C_fabs(ch.CH_C_cos(2 * np.pi * t))
        return self.rot_angle


motor_body = ch.ChBody()
motor_body.SetPos([0, 0, 10])
system.AddBody(motor_body)

motor_function = CustomMotorFunction()
motor_joint = ch.ChLinkMotorRotationAngle()
motor_joint.SetMotorFunction(motor_function)
motor_joint.Initialize(beam_body, motor_body, ch.ChFrame([0, 0, 0], [0, 0, 0]))
system.AddLink(motor_joint)


fixed_constraint = ch.ChLinkFixed()
fixed_constraint.Initialize(beam_body, motor_body)
system.AddLink(fixed_constraint)


system.SetSolverType(ch.ChSolverType_BARZETTI)
system.SetTimestepperType(ch.ChTimestepperType.EULER_IMPLICIT_LINEARIZED)


vis = ch.ChVisualization()
vis.SetSymbolScale(0.1)
beam_body.AddVisualizationShape(vis)


app = ch.ChIrrApp(system, "Beam Buckling Simulation")


app.SetTimestep(0.01)
app.SetTryRealtime(True)

while app.GetShouldKeepRunning():
    app.DoStep()
    app.Render()

app.EndSimulation()