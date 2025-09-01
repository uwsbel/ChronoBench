import pychrono

import pychrono

import pychrono.Irrlicht

import pychrono.ANCF




chrono = pychrono.ChSystemNSC()

irr = pychrono.Irrlicht()




chrono.AddGravity(chrono.Vector(0, -9.81, 0))




beam_length = 1.0

num_elements = 10

element_length = beam_length / num_elements

beam = pychrono.ANCF.CableBeam(chrono)

beam.SetTotalLength(beam_length)

beam.SetNumElements(num_elements)

beam.SetElementLength(element_length)

beam.SetEA(1000.0)  

beam.SetMassPerUnitLength(1.0)  




beam.AddRigidBody(chrono)




hinge = pychrono.RevoluteJoint(chrono)

hinge.SetAnchor(chrono.ChVector(0, 0, 0))

hinge.body = beam

hinge.axis = chrono.ChVector(0, 1, 0)

beam.AddJoint(hinge)




irr.SetGravity(chrono.ChVector(0, -9.81, 0))

irr.SetWindowTitle("ANCF Beam Simulation")

irr.SetCamera(chrono.ChCameraPos(chrono.ChVector(0, 0, 10), chrono.ChQuaternion(0, 0, 0, 1)))

irr.SetTargetFrameRate(60)




def visualize(body, time):

    if body == beam:

        for i in range(num_elements):

            pos = beam.GetPosition(i)

            print(f"Node {i}: {pos}")

            irr.AddSphere(pos, 0.05, 0, 0, 0)


irr.SetVisualizationCallback(visualize)




while(1):

    chrono.DoStepDynamics(0.01)

    irr.DoStepIrr(0.01)




irr.Run()