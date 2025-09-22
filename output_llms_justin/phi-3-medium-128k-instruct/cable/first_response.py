from pychrono import ChSystemNSC, IrrlichtSpace, ChBeam, ChBody, Vector3d

import pychrono.pychrono as pch

import pychrono.irrlicht as chrono


# Initialize PyChrono environment

ch = ChSystemNSC()

ch.SetSpaceType(IrrlichtSpace)


# Add a hinged ground body

ground = ChBody()

ground.SetBodyType(ChBody::STATIC_MASS)

ground.SetName("Ground")

ch.AddBody(ground)


# Create a beam composed of ANCF cable elements

beam = ChBeam(10, 10, 10)  # length, height, width

beam.SetName("Beam")

beam.SetMass(1000)

beam.SetRestAngle(Vector3d(0, 0, 0))

ch.AddBody(beam)


# Add the beam to the hinged ground

beam.AddAnchor(ground, Vector3d(0, 0, 0))


# Set gravity

ch.SetGravity(Vector3d(0, -9.81, 0))


# Visualize the beam's deformation and nodal positions

def render(body):

    if body.GetName() == "Beam":

        for i in range(beam.GetNumSegments()):

            segment = beam.GetSegment(i)

            p1 = segment.GetStart().GetPos()

            p2 = segment.GetEnd().GetPos()

            print(f"Segment {i}: ({p1.x:.2f}, {p1.y:.2f}, {p1.z:.2f}) to ({p2.x:.2f}, {p2.y:.2f}, {p2.z:.2f})")


# Rendering loop

while(1):

    ch.DoStepDynamics(0.01)

    render(ground)

    render(beam)

    chrono.draw(ch)