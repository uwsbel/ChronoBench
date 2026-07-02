import pychrono as chrono
from pychrono import vehicle, irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
my_system = chrono.ChSystemSMC()
my_system.Set_Gravity(chrono.ChVectorD(0, 0, -9.81))


feda = vehicle.FEDA(my_system)
feda.SetContactMethod(chrono.ChContactMethod_SMC)
feda.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(vehicle.VisualizationType_MESH)
feda.SetSteeringVisualizationType(vehicle.VisualizationType_MESH)
feda.SetWheelVisualizationType(vehicle.VisualizationType_MESH)
feda.SetTireVisualizationType(vehicle.VisualizationType_MESH)


init_pos = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
feda.Initialize(chrono.ChCoordsysD(init_pos, init_rot))


for axle in feda.GetAxles():
    for wheel in [axle.m_wheels[0], axle.m_wheels[1]]:
        wheel.GetTire().SetTireType(vehicle.TireModelType_Rigid)


terrain = vehicle.RigidTerrain(my_system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), chrono.ChVectorD(50, 50, 0))
patch.SetTexture(chrono.GetChronoDataFile('vehicle/terrain/textures/tile4.jpg'), 50, 50)
terrain.Initialize()


app = chronoirr.ChIrrApplication(my_system, 'FEDA Vehicle Simulation', chronoirr.dimension2du(800, 600))
app.SetSkyBox()
app.SetCamera(chronoirr.vector3df(0, 0, 1.5), chronoirr.vector3df(0, 0, 0.5))
app.SetCameraVertical(chronoirr.CameraVerticalDir_Z)
app.SetAmbientLight(chronoirr.SColorf(1, 1, 1, 1))
app.EnableShadows(True)
app.AddTypicalLights()
app.SetTargetFPS(50)


driver = vehicle.ChInteractiveDriverIRR(app, feda.GetSteering(), feda.GetSuspension(), feda.GetTire())
driver.SetSteeringClamp(1.0)
driver.SetThrottleClamp(1.0)
driver.SetBrakingClamp(1.0)
driver.Initialize()


time_step = 1 / 50
while app.GetDevice().run():
    
    chassis_pos = feda.GetChassisBody().GetPos()
    app.GetActiveCamera().setPosition(chronoirr.vector3df(chassis_pos.x - 5, chassis_pos.y, chassis_pos.z + 1.5))
    app.GetActiveCamera().setTarget(chronoirr.vector3df(chassis_pos.x, chassis_pos.y, chassis_pos.z))
    
    
    driver.Update()
    
    
    my_system.DoStepDynamics(time_step)
    feda.Update()
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()