import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)






vehicle_data = veh.M113()
vehicle_data.driveline_type = 'RWD'
vehicle_data.engine_type = 'SHAFTS'
vehicle_data.transmission_type = 'AUTOMATIC_SHAFTS'


vehicle = veh.Vehicle(system, vehicle_data)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


tire_lf = veh.TMeasyTire("FrontLeft")
tire_rf = veh.TMeasyTire("FrontRight")
tire_lr = veh.TMeasyTire("RearLeft")
tire_rr = veh.TMeasyTire("RearRight")

for tire in [tire_lf, tire_rf, tire_lr, tire_rr]:
    tire.SetTireParameters(0.35, 0.11, 0.8, 0.4)
    tire.Initialize(vehicle.GetWheelBody(veh.VI_LEFT), veh.VI_LEFT)
    tire.SetVisualizationMesh(chrono.GetChronoDataFile("tire/tire.obj"))
    system.Add(tire)






terrain_mat = chrono.ChMaterialSurfaceNSC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system, vehicle_data.GetContactMaterial())
patch = terrain.AddPatch(terrain_mat, chrono.ChVectorD(0, 0, 0), 
                         chrono.ChVectorD(0, 0, 1), 200, 100)
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


bmw_logo = chrono.ChVisualShapeBox()
bmw_logo.SetSize(chrono.ChVectorD(1.5, 0.01, 0.75))
bmw_logo.SetTexture(chrono.GetChronoDataFile("textures/bmw_logo.jpg"))
patch.GetGroundBody().AddVisualShape(bmw_logo, chrono.ChFrameD(chrono.ChVectorD(10, 0, 0.01)))

terrain.Initialize()






driver = veh.InteractiveDriver(vehicle.GetVehicle())
driver.SetSteeringGain(0.4)
driver.SetThrottleGain(0.5)
driver.SetBrakingGain(0.4)






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.AttachVehicle(vehicle)
vis.SetWindowTitle('BMW E90 Dynamics Simulation')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddTypicalLights(
    chrono.ChVectorD(-30, -30, 80),
    chrono.ChVectorD(30, 50, 80),
    250, 130
)
vis.AddLightWithShadow(
    chrono.ChVectorD(10, 20, 40),
    chrono.ChVectorD(0, 0, 0),
    150, 10, 60, 2, 512, 
    chrono.ChColor(1.0, 1.0, 0.9)
)
vis.EnableShadows()






step_size = 0.005
render_step = 1.0 / 50  


time = 0
while vis.Run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step_size)

print("Simulation completed successfully.")