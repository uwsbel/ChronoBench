import chrono
import chrono.collision as col
import chrono.sensor as sens
import chrono.postprocess as postprocess
import chrono.utility as util


myChrono = chrono.ChronoPy()
myData = myChrono.GetDataFileManager()
myUtilities = myChrono.GetUtilities()
myCollision = myChrono.GetCollisionSystem()
mySensorManager = myChrono.GetSensorManager()
myPostProcess = myChrono.GetPostProcess()


mySystem = myChrono.CreateSimulationSystem2D()
mySystem.SetSolverType(chrono.ChSolverType.SOLVER_LCP_DANTZIG)
mySystem.SetSolverMaxIterations(100)
mySystem.SetSolverTolerance(1e-5)
mySystem.SetSolverMinIterations(10)
mySystem.SetSolverMinIterations(10)
mySystem.SetMaxPenetrationRecoverySpeed(1.0)


myBody = myUtilities.CreateChBodyEasy(myChrono, myData.GetMediaPath() + "meshes/triangular_mesh.obj", chrono.ChMaterialSurface.METAL_STEEL)
myBody.SetPos(chrono.ChVectorD(0, 0, 0))
mySystem.AddBody(myBody)


myCameraSensor = sens.ChCameraSensor()
myCameraSensor.Create(myChrono, myBody, chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 0, 0), 60, 4.0, 0.1, 1000, True)
myCameraSensor.SetNoiseFilter(sens.ChSensorNoiseFilter.FILTER_GAUSSIAN)
myCameraSensor.SetVisualization(sens.ChSensorVisualization.VISUALIZATION_IMAGE)
mySensorManager.AddSensor(myCameraSensor)


myBody.SetPos(chrono.ChVectorD(0, 0, 0))
myBody.SetFixed(True)


mySystem.Setup()
mySystem.Initialize()
mySystem.SetTimeStep(1e-4)

for i in range(1000):
    mySystem.Update(1e-4)

    
    myCameraSensor.SetPos(chrono.ChVectorD(2 * chrono.cos(i * 0.01), 1, 2 * chrono.sin(i * 0.01)))

    
    print(myCameraSensor.GetImageBuffer())

mySystem.Finalize()