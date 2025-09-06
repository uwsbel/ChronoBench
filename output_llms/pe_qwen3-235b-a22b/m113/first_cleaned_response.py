import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("C:/Chrono/Data/")


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)      
patch_mat.SetRestitution(0.01)  
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                         200, 100)  
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()


m113 = veh.M113(sys)  
m113.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(0, 1, 0),  
    chrono.QuatFromAngleZ(0)     
))
m113.Initialize()

m113.SetChassisVisualizationType(veh.VisualizationType_MESH)
m113.SetSprocketVisualizationType(veh.VisualizationType_MESH)
m113.SetIdlerVisualizationType(veh.VisualizationType_MESH)
m113.SetRoadWheelVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChIrrGuiDriver(m113)  
driver.Initialize()                


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('M113 Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, -6))  
vis.AddTypicalLights()


time_step = 0.01  


while vis.Run():
    time = sys.GetChTime()  
    
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    m113.Synchronize(time, driver.GetInputs(), terrain)
    
    
    sys.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()