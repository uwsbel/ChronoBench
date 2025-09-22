import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import csv




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')





initLocTruck  = chrono.ChVector3d(-10,  0, 0.5)
initRotTruck  = chrono.ChQuaterniond(1, 0, 0, 0)


initLocSedan  = chrono.ChVector3d(  0, -10, 0.5)
initRotSedan  = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE


tire_model_truck = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_TMEASY


contact_method = chrono.ChContactMethod_NSC


step_size        = 1e-3
render_fps       = 50
render_step_size = 1.0 / render_fps
render_steps     = math.ceil(render_step_size / step_size)




truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetInitPosition(chrono.ChCoordsysd(initLocTruck, initRotTruck))
truck.Initialize()


truck.SetChassisVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type)
truck.SetWheelVisualizationType(vis_type)
truck.SetTireVisualizationType(vis_type)


for axle in range(truck.GetNumberAxles()):
    for side in [0, 1]:
        truck.InitializeTire(tire_model_truck, axle, side)


truck.GetSystem().SetCollisionSystemType(chrono.CollisionSystemType.BULLET)




sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetInitPosition(chrono.ChCoordsysd(initLocSedan, initRotSedan))
sedan.Initialize()


sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)


for axle in range(sedan.GetNumberAxles()):
    for side in [0, 1]:
        sedan.InitializeTire(tire_model_sedan, axle, side)

sedan.GetSystem().SetCollisionSystemType(chrono.CollisionSystemType.BULLET)




class MeshTerrain:
    def __init__(self, system):
        
        mesh = chrono.ChTriangleMeshConnected()
        mesh.LoadWavefrontMesh(veh.GetDataFile('terrain/meshes/highway.obj'),
                               False, False)
        
        body = chrono.ChBody()
        body.SetName('highway')
        body.SetBodyFixed(True)
        body.SetCollide(True)
        
        mat = chrono.ChMaterialSurfaceNSC()
        mat.SetFriction(0.9)
        body.SetMaterialSurface(mat)
        
        shape = chrono.ChTriangleMeshShape()
        shape.SetMesh(mesh)
        shape.SetBackfaceCull(True)
        body.AddAsset(shape)
        
        c = chrono.ChColorAsset()
        c.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        body.AddAsset(c)
        
        system.Add(body)
        self.body = body

    def Synchronize(self, time):
        
        pass

    def Advance(self, step):
        pass

terrain = MeshTerrain(truck.GetSystem())




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('KrAZ + Sedan Demo')
vis.SetWindowSize(1280, 1024)

vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.1), 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()

vis.AttachVehicle(truck.GetTractor())
vis.AttachVehicle(sedan)





driver_truck = veh.ChInteractiveDriverIRR(vis)
driver_truck.SetSteeringDelta(render_step_size / 1.0)
driver_truck.SetThrottleDelta(render_step_size / 1.0)
driver_truck.SetBrakingDelta(render_step_size / 0.3)
driver_truck.Initialize()


class FixedDriver:
    def __init__(self, throttle, steering):
        self.inputs = veh.DriverInputs()
        self.inputs.m_throttle = throttle
        self.inputs.m_braking  = 0.0
        self.inputs.m_steering = steering

    def Synchronize(self, time):
        pass

    def Advance(self, step):
        pass

    def GetInputs(self):
        return self.inputs

driver_sedan = FixedDriver(throttle=0.5, steering=0.0)




print("TRUCK MASS  = ", truck.GetTractor().GetMass())
print("SEDAN MASS  = ", sedan.GetMass())

realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0


tractor_log = []
trailer_log = []




while vis.Run():
    time = truck.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    in_tr = driver_truck.GetInputs()
    in_sd = driver_sedan.GetInputs()

    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, in_tr, terrain)
    sedan.Synchronize(time, in_sd, terrain)
    vis.Synchronize(time, in_tr)

    
    PT = truck.GetTractor()
    pT = PT.GetPos()
    rT = PT.GetRot()
    tractor_log.append((time, pT.x, pT.y, pT.z, rT.e0, rT.e1, rT.e2, rT.e3))
    TR = truck.GetTrailer()
    if TR:
        pR = TR.GetPos()
        rR = TR.GetRot()
        trailer_log.append((time, pR.x, pR.y, pR.z, rR.e0, rR.e1, rR.e2, rR.e3))

    
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)




with open('tractor_states.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['time','x','y','z','qw','qx','qy','qz'])
    w.writerows(tractor_log)

with open('trailer_states.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['time','x','y','z','qw','qx','qy','qz'])
    w.writerows(trailer_log)