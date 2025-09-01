import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# 1. Initialize PyChrono environment
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Create vehicle system
vehicle = veh.MAN_5t_Vehicle()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_PRIMITIVES)
vehicle.SetChassisFixed(False) 
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.Initialize()

# Adjust vehicle parameters for 10t configuration
vehicle.GetVehicle().SetChassisMass(10000)  # 10,000 kg
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# 3. Create and initialize TMEASY tires
tire_pressure = 130e3  # Pa
for axle in vehicle.GetVehicle().GetAxles():
    tire_l = veh.TMeasyTire(axle.m_wheels[0].GetName())
    tire_r = veh.TMeasyTire(axle.m_wheels[1].GetName())
    
    tire_l.SetPressure(tire_pressure)
    tire_r.SetPressure(tire_pressure)
    
    tire_l.Initialize(axle.m_wheels[0])
    tire_r.Initialize(axle.m_wheels[1])
    
    system.Add(tire_l)
    system.Add(tire_r)

# 4. Create rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 200, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# 5. Set up Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.Initialize()

# Add chase camera
vis.AddCamera(chrono.ChVectorD(-6, 0, 2), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(-5, 0, 8), chrono.ChVectorD(0, 0, 0), 50, 5, 50, 35, 512)
vis.AddSkyBox()
vis.EnableShadows()

# 6. Create real-time driver controls
driver = veh.ChIrrGuiDriver(vis)
steering_time = 1.0  # Time to turn from lock to lock
driver.SetSteeringDelta(1.0 / (50.0 * steering_time))
driver.SetThrottleDelta(1.0 / 50.0)
driver.SetBrakingDelta(1.0 / 50.0)

# 7. Simulation loop parameters
step_size = 0.02
render_step = 1.0 / 50  # FPS

# Main simulation loop
time = 0
while vis.Run():
    time = system.GetChTime()
    
    # Update driver inputs
    driver_inputs = driver.GetInputs()
    vehicle.GetVehicle().Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    # Advance simulation
    driver.Synchronize(time)
    vis.Synchronize(time, driver_inputs)
    system.DoStepDynamics(step_size)
    
    # Update visualization
    if time % render_step < step_size:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Update chase camera
    cam_pos = vehicle.GetVehicle().GetChassisBody().GetPos() + chrono.ChVectorD(-6, 0, 2)
    vis.GetActiveCamera().setTarget(irr.vector3df(vehicle.GetVehicle().GetChassisBody().GetPos().x,
                                                 vehicle.GetVehicle().GetChassisBody().GetPos().y,
                                                 vehicle.GetVehicle().GetChassisBody().GetPos().z))