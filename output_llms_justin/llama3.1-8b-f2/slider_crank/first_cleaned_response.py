import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.cascade as cascade
from OCC.Core import BRepPrimAPI
from OCC.Core import BRepAlgoAPI
from OCC.Core import TopoDS
from OCC.Core import BRepAlgoAPI
import math
import os







change_appearance = False










cylinder = BRepPrimAPI.BRepPrimAPI_MakeCylinder(0.1,0.02).Shape()
axle_hole = BRepPrimAPI.BRepPrimAPI_MakeCylinder(0.005,0.01).Shape()
solid = BRepAlgoAPI.BRepAlgoAPI_Cut(cylinder, axle_hole).Shape()
crankshaft = TopoDS.TopoDS_Shape(solid)
crank = cascade.ChBodyEasyBox(0.02, 0.1, 0.02, 1000, True, True)
crank.SetPos(chrono.ChVector3d(0,0.3,0))
crank.SetFixed(False)
crank.SetRot(chrono.ChQuaterniond(1,0,0,0))
moteur = cascade.ChLinkMotorRotationAngle()
moteur.SetSpindleConstraint(cascade.ChConstraintSpindle(crank, chrono.ChFramed(chrono.ChVector3d(0,0.325,0),chrono.ChQuaterniond(1,0,0,0))))
moteur.SetPrimitifDriver(cascade.ChPrimitifDriver_ANGLE, cascade.ChPeriodicDriver(2,math.pi))
crank.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


rod = cascade.ChBodyEasyBox(0.01, 0.15, 0.01, 1000, True, True)
rod.SetPos(chrono.ChVector3d(0,-0.3,0))
rod.SetFixed(False)
rod.SetRot(chrono.ChQuaterniond(1,0,0,0))


piston = cascade.ChBodyEasyBox(0.02, 0.01, 0.02, 1000, True, True)
piston.SetPos(chrono.ChVector3d(0,-0.6,0))
piston.SetFixed(False)
piston.SetRot(chrono.ChQuaterniond(1,0,0,0))


floor = cascade.ChBodyEasyBox(2,1,0.1, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0,0,0.05))
floor.SetFixed(True)
floor.SetRot(chrono.ChQuaterniond(1,0,0,0))






vehicle = cascade.ChCascadeVehicle(crank, (0, 0, 0), (1, 0, 0, 0))
vehicle.SetDriveline(moteur.GetSpindleConstraint().GetChBodyA(), crank.GetChBody(), cascade.DRIVELINE_TYPE_SHAFTS)
vehicle.Initialize()


vehicle.SetTireType(cascade.ChCascadeTire.TIRE_TYPE_RIGID, None, None)


vehicle.Initialize()






floor.GetCollisionModel().SetCollisionType(chrono.ChCollisionModel.Type_BULLET)


floor.GetVisualShape(0).SetVisualizationType(chronoirr.VisualizationType_PRIMITIVES)


floor.SetFixed(True)


cascade.SetCollisionMargin(crank, 0.01)
cascade.SetCollisionMargin(rod, 0.01)
cascade.SetCollisionMargin(piston, 0.01)
cascade.SetCollisionMargin(floor, 0.01)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(crank.GetSystem())
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1024,720)
vis.SetWindowTitle('Simple mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5,0.5,1.5), chrono.ChVector3d(0,0,0))
vis.AddTypicalLights()






print( "VEHICLE MASS: ",  vehicle.GetMass())


steps_between_events = int(round(1.0 / time_step))


step_number = 0


t = 0.0

while vis.Run() :
    time = vehicle.GetChTime()

    
    if (time >= 1.2):
        break

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = vehicle.GetDriverInputs()

    
    vehicle.Synchronize(time)
    rod.Synchronize(time)
    piston.Synchronize(time)
    floor.Synchronize(time)
    vis.Synchronize(time, driver_inputs)

    
    vehicle.Advance(step_size)
    rod.Advance(step_size)
    piston.Advance(step_size)
    floor.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

return 0