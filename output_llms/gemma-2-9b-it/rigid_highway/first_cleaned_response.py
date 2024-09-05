import pychrono as chrono
import pychrono.irrlicht as vis
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data'))
system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.Set_Speed_limit(100)  
vis.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data'))
vis.SetRealTime(True)
vis.SetFrameRate(50)


app = vis.ChIrrApp(system, 'HMMWV Simulation', vis.dimension2du(800, 600))
vis.AddCamera(app.GetSceneManager(), chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(0, 0, 0))


terrain_col = chrono.ChMeshTerrain(os.path.join(os.path.dirname(__file__), 'Highway_col.obj'))
terrain_vis = chrono.ChMeshTerrain(os.path.join(os.path.dirname(__file__), 'Highway_vis.obj'))


terrain_body = chrono.ChBodyEasy(terrain_col, terrain_vis)
terrain_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(terrain_body)


vehicle_model = chrono.ChBodyEasy(os.path.join(os.path.dirname(__file__), 'HMMWV_model.obj'))
vehicle_model.SetPos(chrono.ChVectorD(0, 0, 0.5))
vehicle_model.SetRot(chrono.Q_from_Euler(chrono.ChVectorD(0, 0, 0)))
system.Add(vehicle_model)


vehicle_model.SetWheelModel(chrono.ChWheelModelTMEASY())
vehicle_model.GetWheelModel().SetFriction(0.8)
vehicle_model.GetWheelModel().SetRollingResistance(0.01)


def on_key_pressed(key):
    if key == vis.KEY_LEFT:
        vehicle_model.SetWheelCmd(0, -1)
    elif key == vis.KEY_RIGHT:
        vehicle_model.SetWheelCmd(0, 1)
    elif key == vis.KEY_UP:
        vehicle_model.SetWheelCmd(0, 1)
    elif key == vis.KEY_DOWN:
        vehicle_model.SetWheelCmd(0, -1)

app.SetKeyCallback(on_key_pressed)


while not app.GetDevice().isQuitting():
    app.BeginScene()
    system.DoStepDynamics(1/50.0)
    app.DrawAll()
    app.EndScene()

app.GetDevice().run()