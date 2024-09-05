import pychrono




chrono = pychrono.ChSystemNSC()




gravity = pychrono.ChBody()

chrono.Add(gravity)

chrono.SetGravity(pychrono.ChVectorD(0, -9.81, 0))




sphere = pychrono.ChBody()

sphere.SetMass(1.0)

sphere.SetPos(pychrono.ChVectorD(0, 10, 0))

sphere.SetRot(pychrono.ChVectorD(0, 0, 0))

chrono.Add(sphere)




ground = pychrono.ChBody()

ground.SetStatic(True)

ground.SetMass(0)

ground.SetPos(pychrono.ChVectorD(0, 0, 0))

ground.SetGeomFixes(pychrono.ChEntityIndexSet(sphere.index))

chrono.Add(ground)




sim_time = 10.0  

time_step = 0.01  




chrono.DoSteps(int(sim_time / time_step))