import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math




chrono.SetChronoDataPath(chrono.GetChronoDataPath())      
veh.SetDataPath(chrono.GetChronoDataPath() + "/vehicle/")





initLoc = chrono.ChVector3d(0, 0, 1.0)

initRot = chrono.ChQuaterniond(1, 0, 0, 0)




vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY




contact_method = chrono.ChContactMethod_NSC
contact_vis = False
step_size = 5e-4              
tire_step_size = step_size
render_step_size = 1.0 / 100  




steering_time = 5.0   




vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()


for m in ("Chassis", "Suspension", "Steering", "Wheel", "Tire"):
    getattr(vehicle, f"Set{m}VisualizationType")(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystemType_BULLET)





patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

mesh_file = veh.GetDataFile("terrain/meshes/highway.obj")

class MeshTerrain:
    def __init__(self, system, mesh_filename, mat):
        self.system = system
        
        body = chrono.ChBody()
        body.SetBodyFixed(True)
        body.SetCollide(True)
        
        body.GetCollisionModel().ClearModel()
        tri_mesh = chrono.ChTriangleMeshConnected()
        tri_mesh.LoadWavefrontMesh(mesh_filename,     
                                  False,  
                                  False)  
        body.GetCollisionModel().AddTriangleMesh(
            mat, tri_mesh, False, False,
            chrono.ChVectorD(0, 0, 0),
            chrono.ChMatrix33D(1),
            0.0)
        body.GetCollisionModel().BuildModel()
        
        mesh_shape = chrono.ChTriangleMeshShape()
        mesh_shape.SetMesh(tri_mesh)
        mesh_shape.SetName("highway")
        body.AddAsset(mesh_shape)
        body.AddAsset(chrono.ChColorAsset(0.8, 0.8, 0.8))
        system.Add(body)

    def Synchronize(self, time):
        pass

    def Advance(self, step):
        pass

terrain = MeshTerrain(vehicle.GetSystem(), mesh_file, patch_mat)




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Sedan")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(-5, 0, 1.8), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())




driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / steering_time)

driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()




ref_speed = 20.0  
Kp = 0.1
Ki = 0.01
Kd = 0.0
speed_error_prev = 0.0
speed_integral = 0.0




print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    inp = driver.GetInputs()

    
    current_speed = vehicle.GetVehicle().GetSpeed()
    err = ref_speed - current_speed
    speed_integral += err * step_size
    deriv = (err - speed_error_prev) / step_size
    speed_error_prev = err
    throttle_cmd = Kp * err + Ki * speed_integral + Kd * deriv
    
    throttle_cmd = max(0.0, min(1.0, throttle_cmd))
    inp.m_throttle = throttle_cmd
    
    inp.m_braking = 0.0

    
    driver.Synchronize(t)
    terrain.Synchronize(t)
    vehicle.Synchronize(t, inp, terrain)
    vis.Synchronize(t, inp)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)