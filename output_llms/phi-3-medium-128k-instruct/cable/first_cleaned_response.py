from pychrono import ChSystemNSC, IrrlichtSpace, ChBeam, ChBody, Vector3d

import pychrono.pychrono as pch

import pychrono.irrlicht as chrono




ch = ChSystemNSC()

ch.SetSpaceType(IrrlichtSpace)




ground = ChBody()

ground.SetBodyType(ChBody::STATIC_MASS)

ground.SetName("Ground")

ch.AddBody(ground)




beam = ChBeam(10, 10, 10)  

beam.SetName("Beam")

beam.SetMass(1000)

beam.SetRestAngle(Vector3d(0, 0, 0))

ch.AddBody(beam)




beam.AddAnchor(ground, Vector3d(0, 0, 0))




ch.SetGravity(Vector3d(0, -9.81, 0))




def render(body):

    if body.GetName() == "Beam":

        for i in range(beam.GetNumSegments()):

            segment = beam.GetSegment(i)

            p1 = segment.GetStart().GetPos()

            p2 = segment.GetEnd().GetPos()

            print(f"Segment {i}: ({p1.x:.2f}, {p1.y:.2f}, {p1.z:.2f}) to ({p2.x:.2f}, {p2.y:.2f}, {p2.z:.2f})")




while(1):

    ch.DoStepDynamics(0.01)

    render(ground)

    render(beam)

    chrono.draw(ch)