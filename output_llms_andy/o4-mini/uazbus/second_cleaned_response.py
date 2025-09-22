import pychrono.core as chrono
import pychrono.vehicle as veh
import math




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")




step_size       = 1e-3
render_fps      = 50
render_step     = 1.0 / render_fps
terrain_length  = 100.0
terrain_width   = 100.0





init_loc = chrono.ChVector3d(-40, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)




vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(step_size)

vehicle.Initialize()


vis_type = veh.VisualizationType_MESH
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystemType_BULLET)

print("  VEHICLE MASS = ", vehicle.GetVehicle().GetMass())




patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrain_length, terrain_width)


patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS Double‐Lane‐Change Demo")
vis.SetWindowSize(1280, 1024)

vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())




def get_driver_inputs(t):
    
    if t < 2.0:
        return 0.0, 0.5, 0.0
    elif t < 4.0:
        return  0.8, 0.5, 0.0
    elif t < 6.0:
        return -0.8, 0.5, 0.0
    elif t < 8.0:
        return  0.8, 0.5, 0.0
    elif t < 10.0:
        return 0.0, 0.5, 0.0
    else:
        return 0.0, 0.0, 1.0


render_steps = math.ceil(render_step / step_size)


timer = chrono.ChRealtimeStepTimer()

step_number = 0




while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    steer, thr, brk = get_driver_inputs(t)

    
    inputs = veh.DriverInputs()
    inputs.m_steering = steer
    inputs.m_throttle = thr
    inputs.m_braking  = brk

    
    terrain.Synchronize(t)
    vehicle.Synchronize(t, inputs, terrain)
    vis.Synchronize(t, inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    timer.Spin(step_size)
    step_number += 1