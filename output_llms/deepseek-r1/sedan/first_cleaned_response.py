import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'data/'))
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))




vehicle = veh.SedanVehicle()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_MESH)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)




tire_l = veh.TMeasyTire("FrontLeft")
tire_r = veh.TMeasyTire("FrontRight")
tire_parameters = {
    "mass": 15.0,
    "width": 0.235,
    "rolling_resistance": 0.015,
    "stiffness": 182000,
    "damping": 300
}

for tire in [tire_l, tire_r]:
    tire.SetParameters(tire_parameters)
    tire.Initialize(vehicle.GetVehicle().GetAxle(0).m_wheels[0].spindle, veh.LEFT)
    tire.Initialize(vehicle.GetVehicle().GetAxle(0).m_wheels[1].spindle, veh.RIGHT)
    system.Add(tire)




terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                        chrono.ChVectorD(0, 0, 0), 
                        chrono.ChVectorD(0, 0, 1), 
                        200, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
terrain.Initialize()




vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("BMW E90 Dynamics Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, -10, 1.5), chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLights()


tracker = irr.ChChaseCamera(chrono.ChVectorD(0, 0, 0), 6.0, 0.5)
tracker.AttachVehicle(vehicle.GetVehicle())
vis.AddChaseCamera(tracker)




driver = veh.ChInteractiveDriverIRR(vis)
steering_controller = veh.ChSteeringControllerSMC()
driver.SetSteeringController(steering_controller)
driver.SetInputDataFile("")
driver.SetInputMode(veh.InputMode_DATAFILE)
driver.Initialize()




while vis.Run():
    time = system.GetChTime()
    vis.BeginScene()
    vis.Render()
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    vis.Synchronize("", driver.GetInputs())
    system.DoStepDynamics(1e-3)
    vis.EndScene()