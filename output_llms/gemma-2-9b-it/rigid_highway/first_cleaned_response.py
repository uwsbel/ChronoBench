import pychrono as chrono
import pychrono.irrlicht as vis
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data'))
physics = chrono.ChSystemNSC()


vis_app = vis.ChIrrApp(physics)
vis_app.AddTypicalSky()
vis_app.AddTypicalGround()
vis_app.SetCamera(vis.IrrlichtCamera(vis_app.GetSceneManager(), chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0)))


terrain_col = chrono.ChTriangleMeshConnected()
terrain_col.Load(os.path.join(os.path.dirname(__file__), 'Highway_col.obj'))
terrain_vis = chrono.ChTriangleMeshConnected()
terrain_vis.Load(os.path.join(os.path.dirname(__file__), 'Highway_vis.obj'))


terrain = chrono.ChBodyEasyBox(100, 100, 50, 1000)  
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.SetBodyFixed(True)
terrain.SetCollide(terrain_col)
terrain.GetVisualShape().SetTexture(vis_app.GetAssetManager().GetTexture("Highway_vis.jpg"))  














vis_app.SetTargetFPS(50)
while vis_app.Run():
    physics.DoStepDynamics(1/50.0)
    vis_app.Render()