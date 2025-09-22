import pychrono as chrono

import pychrono.visualization as visual

import pychrono.core as core

import pychrono.fea as fea

import pychrono.irrlicht as irrlicht




core.Initialize()




my_sim = core.Simulation()




iga_beam = fea.IsogeometricAnalysisBeam()

iga_beam.CreateLinearBeam(core.GetMyChronoInstance(),

                           length=1.0,

                           width=0.1,

                           height=0.01,

                           density=7850.0,

                           elasticity=210e9,

                           poisson_ratio=0.3,

                           youngs_modulus=210e9,

                           use_gravity=False)




my_sim.AddSolid(iga_beam)




flywheel = fea.IsogeometricAnalysisSphere(core.GetMyChronoInstance(),

                                        radius=0.05,

                                        density=7850.0,

                                        elasticity=210e9,

                                        poisson_ratio=0.3,

                                        youngs_modulus=210e9,

                                        use_gravity=False)

flywheel_center = core.GetMyChronoInstance().GetBeamGeometry().GetCenter()

flywheel.SetPosition(flywheel_center)

my_sim.AddSolid(flywheel)




motor = fea.IsogeometricAnalysisCylinder(core.GetMyChronoInstance(),

                                        radius=0.01,

                                        length=0.2,

                                        density=7850.0,

                                        elasticity=210e9,

                                        poisson_ratio=0.3,

                                        youngs_modulus=210e9,

                                        use_gravity=False)

motor_end = core.GetMyChronoInstance().GetBeamGeometry().GetEnd(0)

motor.SetPosition(motor_end)

my_sim.AddSolid(motor)




motor.SetRotationSpeed(2000)




vis = visual.ChronoViz()

vis.SetWindowSize(1024, 768)

vis.SetWindowTitle('Jeffcott Rotor Simulation')

vis.SetViewportSize(0.5, 0.5)




my_sim.SetupSteadyStateAnalysis(1.0)

my_sim.SetSolverType(core.CHACORICHON)

my_sim.SetSolverTolerance(1e-5)

my_sim.SetSolverMaxIterations(1000)

my_sim.Solve()




vis.StartRecording()

my_sim.Solve()

vis.StopRecording()

vis.Update()




my_sim.SetupDynamicAnalysis(1.0)

my_sim.SetSolverType(core.CHACORICHON)

my_sim.SetSolverTolerance(1e-5)

my_sim.SetSolverMaxIterations(1000)

my_sim.Solve(10000)




vis.StartRecording()

my_sim.Solve()

vis.StopRecording()

vis.Update()




irrlicht_application = irrlicht.IrrlichtApplication(vis)

irrlicht_application.Run()