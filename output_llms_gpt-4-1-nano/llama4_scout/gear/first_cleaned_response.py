import chrono as ch
import chrono.irrlicht as chirr


ch.ChEngine.Initialize()


app = chirr.ChIrrApp()


app.SetWindowTitle('Epicyclic Gear Simulation')


app.EnableShadows()


truss_body = ch.ChBody()
truss_body.SetBodyFixed(True)


truss_body.SetPos(ch.ChVector3d(0, 0, 0))


app.GetSystem().Add(truss_body)


bar_body = ch.ChBody()
bar_body.SetMass(1)


bar_body.SetPos(ch.ChVector3d(1, 0, 0))
bar_body.SetInertiaXX(ch.ChVector3d(1, 1, 1))


app.GetSystem().Add(bar_body)


bar_joint = ch.ChRevoluteJoint(bar_body, truss_body, ch.ChVector3d(1, 0, 0), ch.ChVector3d(0, 0, 0), ch.ChVector3d(0, 0, 1))
app.GetSystem().Add(bar_joint)


gear1_body = ch.ChBody()
gear1_body.SetMass(5)

gear2_body = ch.ChBody()
gear2_body.SetMass(10)


gear1_body.SetPos(ch.ChVector3d(0.5, 0.5, 0))
gear1_body.SetInertiaXX(ch.ChVector3d(1, 1, 1))

gear2_body.SetPos(ch.ChVector3d(0.5, -0.5, 0))
gear2_body.SetInertiaXX(ch.ChVector3d(1, 1, 1))


app.GetSystem().Add(gear1_body)
app.GetSystem().Add(gear2_body)


gear_motor = ch.ChGearMotor(gear1_body, ch.ChVector3d(0, 0, 1), 10)
app.GetSystem().Add(gear_motor)


gear_interaction = ch.ChGear(gear1_body, gear2_body, 2, 1)
app.GetSystem().Add(gear_interaction)


material = ch.ChVisualMaterial()
material.SetDiffuseColor(ch.ChColor(1, 0, 0))


truss_body.GetVisualShape().SetMaterial(material)
bar_body.GetVisualShape().SetMaterial(material)
gear1_body.GetVisualShape().SetMaterial(material)
gear2_body.GetVisualShape().SetMaterial(material)


app.EnableLighting()


app.GetSystem().SetTimeStep(0.01)


app.GetSystem().SetEndTime(10)


app.Run()

import chrono as ch
import chrono.irrlicht as chirr

def main():
    
    ch.ChEngine.Initialize()

    
    app = chirr.ChIrrApp()

    
    app.SetWindowTitle('Epicyclic Gear Simulation')

    
    app.EnableShadows()

    
    truss_body = ch.ChBody()
    truss_body.SetBodyFixed(True)

    
    truss_body.SetPos(ch.ChVector3d(0, 0, 0))

    
    app.GetSystem().Add(truss_body)

    
    bar_body = ch.ChBody()
    bar_body.SetMass(1)

    
    bar_body.SetPos(ch.ChVector3d(1, 0, 0))
    bar_body.SetInertiaXX(ch.ChVector3d(1, 1, 1))

    
    app.GetSystem().Add(bar_body)

    
    bar_joint = ch.ChRevoluteJoint(bar_body, truss_body, ch.ChVector3d(1, 0, 0), ch.ChVector3d(0, 0, 0), ch.ChVector3d(0, 0, 1))
    app.GetSystem().Add(bar_joint)

    
    gear1_body = ch.ChBody()
    gear1_body.SetMass(5)

    gear2_body = ch.ChBody()
    gear2_body.SetMass(10)

    
    gear1_body.SetPos(ch.ChVector3d(0.5, 0.5, 0))
    gear1_body.SetInertiaXX(ch.ChVector3d(1, 1, 1))

    gear2_body.SetPos(ch.ChVector3d(0.5, -0.5, 0))
    gear2_body.SetInertiaXX(ch.ChVector3d(1, 1, 1))

    
    app.GetSystem().Add(gear1_body)
    app.GetSystem().Add(gear2_body)

    
    gear_motor = ch.ChGearMotor(gear1_body, ch.ChVector3d(0, 0, 1), 10)
    app.GetSystem().Add(gear_motor)

    
    gear_interaction = ch.ChGear(gear1_body, gear2_body, 2, 1)
    app.GetSystem().Add(gear_interaction)

    
    material = ch.ChVisualMaterial()
    material.SetDiffuseColor(ch.ChColor(1, 0, 0))

    
    truss_body.GetVisualShape().SetMaterial(material)
    bar_body.GetVisualShape().SetMaterial(material)
    gear1_body.GetVisualShape().SetMaterial(material)
    gear2_body.GetVisualShape().SetMaterial(material)

    
    app.EnableLighting()

    
    app.GetSystem().SetTimeStep(0.01)

    
    app.GetSystem().SetEndTime(10)

    
    app.Run()

if __name__ == '__main__':
    main()