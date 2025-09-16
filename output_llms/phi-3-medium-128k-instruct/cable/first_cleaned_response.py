from pychrono import ChSystem, ChBodyA, ChQuaternionA, ChVectorA, ChCableA

import irrlicht

from pychrono.modelling import ChBeamA




system = ChSystemNSC()




beam = ChBeamA(system)

beam.SetBodyFixed(True)

beam.SetMass(1.0)

beam.SetLength(1.0)

beam.SetWidth(0.1)

beam.SetHeight(0.1)




cable = ChCableA(system)

cable.SetLength(1.0)

cable.SetMass(0.1)

cable.SetDensity(1000.0)

cable.SetStiffness(10000.0)




hinge = ChRigidBodyA(system)

hinge.SetMass(0.0)

hinge.SetPosition(ChVectorA(0.0, 0.0, 0.0))

hinge.SetRotation(ChQuaternionA(1.0, 0.0, 0.0, 0.0))




cable.Attach(hinge, ChVectorA(0.0, 0.0, 0.0), ChVectorA(0.0, 0.0, 0.0), ChVectorA(0.0, 0.0, 0.0))




system.SetGravity(ChVectorA(0.0, 0.0, -9.81))




cable.SetInitialPosition(ChVectorA(0.0, 0.0, 0.0))




cable.SetInitialVelocity(ChVectorA(0.0, 0.0, 0.0))




beam.SetPosition(ChVectorA(0.0, 0.0, 0.0))




beam.SetRotation(ChQuaternionA(1.0, 0.0, 0.0, 0.0))




beam.SetVelocity(ChVectorA(0.0, 0.0, 0.0))




irr = irrlicht.IrrEngine()

driver = irr.createDevice()

driver.initialize(core.get_option("irrlicht.core.path"))

sceneManager = driver.getSceneManager()

rMaterial = sceneManager.createMaterial("rMaterial")

rMaterial.setColor(irr.video.SColor(255, 0, 0, 255))




while driver.run() == irrlicht.core.ESEL_CONTINUE:

    system.DoStepDynamics(0.01)

    beam.SetPosition(ChVectorA(0.0, 0.0, 0.0))

    beam.SetRotation(ChQuaternionA(1.0, 0.0, 0.0, 0.0))


    

    beam.Draw(rMaterial)

    cable.Draw(rMaterial)




driver.drop()