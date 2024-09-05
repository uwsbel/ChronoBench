import chrono
from chrono import *
import chrono.irrlicht as ci


irr = ci.ChIrrApp(chrono.GetChronoDataPath(), "HMMWV on SCM Terrain")
irr.AddTypicalSky()
irr.AddTypicalLogo(chrono.GetChronoDataPath())
irr.SetCamera(ChVectorD(10, 10, 10), ChVectorD(0, 0, 0))
irr.SetTimestep(0.005)  


my_system = chrono.System()
my_system.Set_Gravity(ChVectorD(0, -9.81, 0))


hmmwv = chrono.vehicle.GetVehicle("HMMWV")
hmmwv.SetChassisVisualizationType(chrono.vehicle.GetVehicle().VISUALIZATION_MESH)
hmmwv.SetChassisPos(ChVectorD(0, 0.5, 0))
hmmwv.SetChassisBody(my_system.Get_bodylist().Append(hmmwv.GetChassisBody()))
my_system.Add(hmmwv)


terrain = chrono.terrain.SCMTerrain()
terrain.SetAssetPath(chrono.GetChronoDataPath())
terrain.SetName("terrain")
terrain.SetSize(100, 100)
terrain.SetMovingPatch(True)
terrain.SetMovingPatchRadius(5)
terrain.SetMovingPatchOffset(ChVectorD(0, 0, 0))
terrain.SetMovingPatchFollowBody(hmmwv.GetChassisBody())
terrain.SetSoilParams(1.8e6, 0.3, 0.1, 0.05, 0.01)
terrain.SetVisualizationType(chrono.terrain.SCMTerrain.VISUALIZATION_FALSCOLOR)
my_system.Add(terrain)


driver = chrono.vehicle.GetVehicleDriver("HMMWV")
driver.SetSteering(0.5)
driver.SetThrottle(0.5)
driver.SetBrake(0)
my_system.Add(driver)


while irr.GetDevice().run():
    my_system.DoStepDynamics(0.005)
    my_system.DoStepGraphics(0.005)
    irr.DrawAll()
    irr.GetDevice().driver.wait()
    irr.GetDevice().swapBuffers()