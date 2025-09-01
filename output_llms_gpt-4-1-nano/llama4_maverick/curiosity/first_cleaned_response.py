import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens


print("Copyright (c) 2023 Project Chrono")


sys = chrono.ChSystemNSC()



terrain = veh.RigidTerrain(sys)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
terrain.SetContactMaterial(chrono.ChMaterialSurface.NSCoulomb)
patch_mat = chrono.ChMaterialSurface.NSCoulomb()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngZ(0)), 20, 20)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 20, 20)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


init_pos = chrono.ChFrameD(chrono.ChVectorD(0, 0.2, -5), chrono.Q_from_AngZ(0))
rover = veh.Curiosity(sys, veh.GetDataFile("curiosity/curiosity.json"), init_pos)


rover.SetChassisFixed(False)
rover.SetWheelCollision(True)
rover.Initialize()



rover.GetSteering(0).SetSteering(-0.3)
rover.GetThrottle().SetThrottle(0.3)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.EnableShadows()


while vis.Run():
    sys.DoStepDynamics(1e-3)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


vis.GetDevice().closeDevice()
print("Simulation Complete")