import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

import math

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# If true, use provided callback to change camera vertical zoom
change_camera_height = True

class MyReportContact(chrono.ReportContactCallback):
    def __init__(self):
        chrono.ReportContactCallback.__init__(self)

    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
        return True        # return False to ignore contact

            # Create the rigd body
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

mm = 1.0
tank = veh.RigidTanks.TC_MESH
chassis_file = chrono.GetChronoDataFile("vehicle/tank/" + ("hmmwv" if tank == veh.RigidTanks.TC_HMMWV else "leopard") + "/chassis_mesh.obj")

chassis_vis_shape = chrono.ChTriangleMeshConnected()
chassis_vis_shape.LoadWavefrontMesh(chassis_file, True, True);

scale = 0.5
chassis_vis_shape.Transform(chrono.ChVector3d(scale, scale, scale), chrono.ChMatrix33d(1))

vis_shape_material = chrono.ChVisualMaterial()
vis_shape_material.SetAmbientColor(chrono.ChColor(0.4, 0.4, 0.4))
vis_shape_material.SetDiffuseColor(chrono.ChColor(0.4, 0.4, 0.4))
vis_shape_material.SetSpecularColor(chrono.ChColor(0.1, 0.1, 0.1))

# Please note that we are not creating a body for the mesh. We will attach it to the chassis later.

chassis_body = veh.RigidChassisBody(
    2000 if tank == veh.RigidTanks.TC_LEOPARD else 2400,
    chrono.ChVector3d(2.0, 0.5, 3.0), # center of gravity
    scale * 1.02, # half track width
    scale * 0.95, # half wheel base
    scale * 1.8, # height
    scale * 0.27, # rear bump
    scale * 0.3, # suspension bump
    0.2, # track shoe width
    0.8, # first track shoe offset
    0.6, # track shoe length
    0.1, # pitch angle
    0.3, # toe angle
    0.3, # wheel radius
    0.2, # wheel width
    0.1, # moogie spring rest length
    0.2, # moogie spring width
    0.2, # moogie shock shaft radius
    0.2, # moogie shock body radius
    0.2, # sprocket radius
    0.2, # idler radius
    0.2, # idler width
    0.2, # return roller radius
    0.2, # return roller width
    0.2, # support roller radius
    0.2, # support roller width
    0.2, # tension roller radius
    0.2, # tension roller width
    0.2, # track segment length
    0.2, # track spring rest length
    0.2, # track shoe spring rest length
    0.2, # track segment width
    0.2, # track thickness
    0.2, # track mass
    0.2, # track stiffness
    0.2, # track damping
    0.2, # track friction
    0.2, # shoe stiffness
    0.2, # shoe damping
    0.2, # sprocket ratio
    0.2, # idler ratio
    0.2, # return roller ratio
    0.2, # support roller ratio
    0.2, # tension roller ratio
    0.2, # sprocket radius
    0.2, # idler radius
    0.2, # return roller radius
    0.2, # support roller radius
    0.2, # tension roller radius
    0.2, # sprocket width
    0.2, # idler width
    0.2, # return roller width
    0.2, # support roller width
    0.2, # tension roller width
    0.2, # sprocket offset
    0.2, # idler offset
    0.2, # return roller offset
    0.2, # support roller offset
    0.2, # tension roller offset
    0.2, # sprocket frame ratio
    0.2, # idler frame ratio
    0.2, # return roller frame ratio
    0.2, # support roller frame ratio
    0.2, # tension roller frame ratio
    0.2, # sprocket suspension spring rest length
    0.2, # idler suspension spring rest length
    0.2, # return roller suspension spring rest length
    0.2, # support roller suspension spring rest length
    0.2, # tension roller suspension spring rest length
    0.2, # sprocket suspension spring stiffness
    0.2, # idler suspension spring stiffness
    0.2, # return roller suspension spring stiffness
    0.2, # support roller suspension spring stiffness
    0.2, # tension roller suspension spring stiffness
    0.2, # sprocket suspension spring damping
    0.2, # idler suspension spring damping
    0.2, # return roller suspension spring damping
    0.2, # support roller suspension spring damping
    0.2, # tension roller suspension spring damping
    0.2, # sprocket shock stiffness
    0.2, # idler shock stiffness
    0.2, # return roller shock stiffness
    0.2, # support roller shock stiffness
    0.2, # tension roller shock stiffness
    0.2, # sprocket shock damping
    0.2, # idler shock damping
    0.2, # return roller shock damping
    0.2, # support roller shock damping
    0.2, # tension roller shock damping
    0.2, # sprocket torsion rest angle
    0.2, # idler torsion rest angle
    0.2, # return roller torsion rest angle
    0.2, # support roller torsion rest angle
    0.2, # tension roller torsion rest angle
    0.2, # sprocket torsion stiffness
    0.2, # idler torsion stiffness
    0.2, # return roller torsion stiffness
    0.2, # support roller torsion stiffness
    0.2, # tension roller torsion stiffness
    0.2, # sprocket torsion damping
    0.2, # idler torsion damping
    0.2, # return roller torsion damping
    0.2, # support roller torsion damping
    0.2, # tension roller torsion damping
    0.2, # sprocket frame stiffness
    0.2, # idler frame stiffness
    0.2, # return roller frame stiffness
    0.2, # support roller frame stiffness
    0.2, # tension roller frame stiffness
    0.2, # sprocket frame damping
    0.2, # idler frame damping
    0.2, # return roller frame damping
    0.2, # support roller frame damping
    0.2, # tension roller frame damping
    0.2, # sprocket vertical stiffness
    0.2, # idler vertical stiffness
    0.2, # return roller vertical stiffness
    0.2, # support roller vertical stiffness
    0.2, # tension roller vertical stiffness
    0.2, # sprocket vertical damping
    0.2, # idler vertical damping
    0.2, # return roller vertical damping
    0.2, # support roller vertical damping
    0.2, # tension roller vertical damping
    0.2, # sprocket longitudinal stiffness
    0.2, # idler longitudinal stiffness
    0.2, # return roller longitudinal stiffness
    0.2, # support roller longitudinal stiffness
    0.2, # tension roller longitudinal stiffness
    0.2, # sprocket longitudinal damping
    0.2, # idler longitudinal damping
    0.2, # return roller longitudinal damping
    0.2, # support roller longitudinal damping
    0.2, # tension roller longitudinal damping
    0.2, # sprocket lateral stiffness
    0.2, # idler lateral stiffness
    0.2, # return roller lateral stiffness
    0.2, # support roller lateral stiffness
    0.2, # tension roller lateral stiffness
    0.2, # sprocket lateral damping
    0.2, # idler lateral damping
    0.2, # return roller lateral damping
    0.2, # support roller lateral damping
    0.2, # tension roller lateral damping
    0.2, # sprocket camber stiffness
    0.2, # idler camber stiffness
    0.2, # return roller camber stiffness
    0.2, # support roller camber stiffness
    0.2, # tension roller camber stiffness
    0.2, # sprocket camber damping
    0.2, # idler camber damping
    0.2, # return roller camber damping
    0.2, # support roller camber damping
    0.2, # tension roller camber damping
    0.2, # sprocket elevation stiffness
    0.2, # idler elevation stiffness
    0.2, # return roller elevation stiffness
    0.2, # support roller elevation stiffness
    0.2, # tension roller elevation stiffness
    0.2, # sprocket elevation damping
    0.2, # idler elevation damping
    0.2, # return roller elevation damping
    0.2, # support roller elevation damping
    0.2, # tension roller elevation damping
    0.2, # sprocket frame elevation stiffness
    0.2, # idler frame elevation stiffness
    0.2, # return roller frame elevation stiffness
    0.2, # support roller frame elevation stiffness
    0.2, # tension roller frame elevation stiffness
    0.2, # sprocket frame elevation damping
    0.2, # idler frame elevation damping
    0.2, # return roller frame elevation damping
    0.2, # support roller frame elevation damping
    0.2, # tension roller frame elevation damping
    0.2, # sprocket frame elevation rest angle
    0.2, # idler frame elevation rest angle
    0.2, # return roller frame elevation rest angle
    0.2, # support roller frame elevation rest angle
    0.2, # tension roller frame elevation rest angle
    0.2, # sprocket frame elevation spring rest length
    0.2, # idler frame elevation spring rest length
    0.2, # return roller frame elevation spring rest length
    0.2, # support roller frame elevation spring rest length
    0.2, # tension roller frame elevation spring rest length
    0.2, # sprocket frame elevation spring stiffness
    0.2, # idler frame elevation spring stiffness
    0.2, # return roller frame elevation spring stiffness
    0.2, # support roller frame elevation spring stiffness
    0.2, # tension roller frame elevation spring stiffness
    0.2, # sprocket frame elevation spring damping
    0.2, # idler frame elevation spring damping
    0.2, # return roller frame elevation spring damping
    0.2, # support roller frame elevation spring damping
    0.2, # tension roller frame elevation spring damping
    0.2, # sprocket frame elevation shock stiffness
    0.2, # idler frame elevation shock stiffness
    0.2, # return roller frame elevation shock stiffness
    0.2, # support roller frame elevation shock stiffness
    0.2, # tension roller frame elevation shock stiffness
    0.2, # sprocket frame elevation shock damping
    0.2, # idler frame elevation shock damping
    0.2, # return roller frame elevation shock damping
    0.2, # support roller frame elevation shock damping
    0.2, # tension roller frame elevation shock damping
    0.2, # sprocket frame elevation torsion rest angle
    0.2, # idler frame elevation torsion rest angle
    0.2, # return roller frame elevation torsion rest angle
    0.2, # support roller frame elevation torsion rest angle
    0.2, # tension roller frame elevation torsion rest angle
    0.2, # sprocket frame elevation torsion stiffness
    0.2, # idler frame elevation torsion stiffness
    0.2, # return roller frame elevation torsion stiffness
    0.2, # support roller frame elevation torsion stiffness
    0.2, # tension roller frame elevation torsion stiffness
    0.2, # sprocket frame elevation torsion damping
    0.2, # idler frame elevation torsion damping
    0.2, # return roller frame elevation torsion damping
    0.2, # support roller frame elevation torsion damping
    0.2, # tension roller frame elevation torsion damping
    0.2, # sprocket frame elevation torsion spring rest length
    0.2, # idler frame elevation torsion spring rest length
    0.2, # return roller frame elevation torsion spring rest length
    0.2, # support roller frame elevation torsion spring rest length
    0.2, # tension roller frame elevation torsion spring rest length
    0.2, # sprocket frame elevation torsion spring stiffness
    0.2, # idler frame elevation torsion spring stiffness
    0.2, # return roller frame elevation torsion spring stiffness
    0.2, # support roller frame elevation torsion spring stiffness
    0.2, # tension roller frame elevation torsion spring stiffness
    0.2, # sprocket frame elevation torsion spring damping
    0.2, # idler frame elevation torsion spring damping
    0.2, # return roller frame elevation torsion spring damping
    0.2, # support roller frame elevation torsion spring damping
    0.2, # tension roller frame elevation torsion spring damping
    0.2, # sprocket frame elevation torsion shock stiffness
    0.2, # idler frame elevation torsion shock stiffness
    0.2, # return roller frame elevation torsion shock stiffness
    0.2, # support roller frame elevation torsion shock stiffness
    0.2, # tension roller frame elevation torsion shock stiffness
    0.2, # sprocket frame elevation torsion shock damping
    0.2, # idler frame elevation torsion shock damping
    0.2, # return roller frame elevation torsion shock damping
    0.2, # support roller frame elevation torsion shock damping
    0.2, # tension roller frame elevation torsion shock damping
    0.2, # sprocket frame elevation camber rest angle
    0.2, # idler frame elevation camber rest angle
    0.2, # return roller frame elevation camber rest angle
    0.2, # support roller frame elevation camber rest angle
    0.2, # tension roller frame elevation camber rest angle
    0.2, # sprocket frame elevation camber stiffness
    0.2, # idler frame elevation camber stiffness
    0.2, # return roller frame elevation camber stiffness
    0.2, # support roller frame elevation camber stiffness
    0.2, # tension roller frame elevation camber stiffness
    0.2, # sprocket frame elevation camber damping
    0.2, # idler frame elevation camber damping
    0.2, # return roller frame elevation camber damping
    0.2, # support roller frame elevation camber damping
    0.2, # tension roller frame elevation camber damping
    0.2, # sprocket frame elevation camber spring rest length
    0.2, # idler frame elevation camber spring rest length
    0.2, # return roller frame elevation camber spring rest length
    0.2, # support roller frame elevation camber spring rest length