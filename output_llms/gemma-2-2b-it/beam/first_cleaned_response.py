import pychrono as pch
import pychrono.physics as pchphys
import pychrono.visualization as pchvis


p = pch.PyChrono()


world = pch.World()


mesh = pch.Mesh()
mesh.SetMeshType(pch.MeshType.BEAM)
mesh.SetMeshFilename("beam_mesh.obj")


beam = pch.BeamElement(mesh)
beam.SetBeamMaterial(pch.Material(0.0, 0.0, 0.0))


node = pch.Node(mesh)
node.SetNodeMaterial(pch.Material(0.0, 0.0, 0.0))


world.AddObject(beam)
world.AddObject(node)


beam.SetPosition(pch.Vector3(0.0, 0.0, 0.0))
node.SetPosition(pch.Vector3(0.0, 0.0, 0.0))


beam.SetForce(pch.Vector3(0.0, 0.0, 0.0))
node.SetForce(pch.Vector3(0.0, 0.0, 0.0))


irr = pch.Irrlicht()


irr.CreateIrrlichtScene()


while True:
    
    world.Update(0.01)

    
    irr.Render()

    
    if input("Press any key to exit: ") == "":
        break