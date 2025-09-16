from pychoro import *

import irrlicht as irr




init(6)




mesh = Mesh()

mesh.addBeam(2, 10, 1000, 100, 1000, 0, 0, 0, 0, 0)




system = System()

system.addMesh(mesh)




system.setGravity([0, -9.81, 0])




scene = Scene()

camera = scene.addCamera("Main Camera", 5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0)

renderer = scene.addRenderer("Main Renderer")




while not system.isStopped():

    system.advance()

    scene.drawAll()

    system.waitEvent()




scene.removeAll()

system.removeAll()