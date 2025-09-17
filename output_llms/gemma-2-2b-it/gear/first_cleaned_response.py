import pychrono as chrono
import pychrono.irrlicht as irrlicht


chrono.initialize()


irr = irrlicht.IrrlichtEngine()


scene = irr.createScene()


truss = chrono.Truss(
    chrono.Vec3(0, 0, 0),
    chrono.Vec3(1, 0, 0),
    chrono.Vec3(0, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
)


bar = chrono.Bar(
    chrono.Vec3(0, 0, 0),
    chrono.Vec3(1, 0, 0),
    chrono.Vec3(0, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
)


gear1 = chrono.Gear(
    chrono.Vec3(0, 0, 0),
    chrono.Vec3(1, 0, 0),
    chrono.Vec3(0, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
)

gear2 = chrono.Gear(
    chrono.Vec3(0, 0, 0),
    chrono.Vec3(1, 0, 0),
    chrono.Vec3(0, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
)


gear_motor = chrono.GearMotor(
    chrono.Vec3(0, 0, 0),
    chrono.Vec3(1, 0, 0),
    chrono.Vec3(0, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
    chrono.Vec3(0, 0, 1),
    chrono.Vec3(1, 1, 0),
)


scene.add(truss)
scene.add(bar)
scene.add(gear1)
scene.add(gear2)
scene.add(gear_motor)


irr.setMaterial(chrono.Material(chrono.Color(1, 0, 0, 1)))
irr.setMaterial(chrono.Material(chrono.Color(0, 1, 0, 1)))
irr.setMaterial(chrono.Material(chrono.Color(0, 0, 1, 1)))


irr.run()


scene.delete()
irr.shutdown()