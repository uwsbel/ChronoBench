import pychrono as pc

import irrlicht as ir

from irrlicht import scene_io




pc.initialize()




scene = pc.Scene()




truss = pc.Truss()

truss.setPosition(pc.Vector3r(0, 0, 0))

scene.add(truss)




bar = pc.Bar()

bar.setPosition(pc.Vector3r(0, 1, 0))

bar.setLength(2)

bar.setRotation(pc.Quaternionr(0, 0, 0, 1))

scene.add(bar)




gear1 = pc.Gear()

gear2 = pc.Gear()

gear1.setPosition(pc.Vector3r(0, 2, 0))

gear2.setPosition(pc.Vector3r(0, 4, 0))

gear1.setMotor(True)

gear2.setMotor(True)

gear1.setMotorSpeed(1.0)  

scene.add(gear1)

scene.add(gear2)




window = ir.IrrlichtGraphicsDevice(400, 400, 24, ir.Dimension.COLOR)

scene_io.writeI3DFile(scene, "epicyclic_gears_scene.i3d", window)




pc.run()