import pychrono                 as chrono
import pychrono.fea             as fea
import pychrono.irrlicht        as chronoirr
import math




chrono.SetChronoDataPath( chrono.GetChronoDataPath() )        
system = chrono.ChSystemNSC()                                  
system.Set_G_acc( chrono.ChVectorD(0,0,0) )                    




E_modulus    = 2.1e11      
poisson      = 0.3
density      = 7800        
diameter     = 0.02        
length       = 0.6         
num_elems    = 16          

section      = fea.ChBeamSectionAdvanced()
section.SetDensity( density )
section.SetYoungModulus( E_modulus )
section.SetPoissonRatio( poisson )
section.SetAsCircularSection( diameter )

mesh   = fea.ChMesh()

builder = fea.ChBuilderBeamIGA()


builder.BuildBeam( mesh,
                   section,
                   num_elems,
                   chrono.ChVectorD( 0, 0, 0 ),
                   chrono.ChVectorD( length, 0, 0 ),
                   chrono.ChVectorD( 0, 1, 0 ) )         


first_node   = builder.GetLastBeamNodes().front()             
mid_node     = builder.GetLastBeamNodes()[ num_elems // 2 ]   
last_node    = builder.GetLastBeamNodes().back()              

system.Add( mesh )




ground   = chrono.ChBody()              
ground.SetBodyFixed( True )
system.Add( ground )


rev_right = chrono.ChLinkLockRevolute()
rev_right.Initialize( last_node, ground,
                      chrono.ChCoordsysD( last_node.GetPos(),
                                          chrono.Q_from_AngAxis( 0, chrono.ChVectorD(0,1,0)) ) )
system.Add( rev_right )


motor_fun = chrono.ChFunctionConstant( 2*math.pi*30 )     
motor     = chrono.ChLinkMotorRotationSpeed()
motor.Initialize( first_node, ground,
                  chrono.ChFrameD( first_node.GetPos(),
                                   chrono.Q_from_AngAxis( 0, chrono.ChVectorD(0,1,0)) ) )
motor.SetSpeedFunction( motor_fun )
system.Add( motor )


fix_left = chrono.ChLinkMateGeneric( True, True, True,    
                                     False, False, False )
fix_left.Initialize( first_node, ground, first_node.Frame() )
system.Add( fix_left )




disc_radius  = 0.08          
disc_thick   = 0.02          
flywheel     = chrono.ChBodyEasyCylinder( disc_radius, disc_thick, density )
flywheel.SetPos( mid_node.GetPos() )
flywheel.SetRot( mid_node.GetRot() )
system.Add( flywheel )


rigid_link   = chrono.ChLinkMateFix()
rigid_link.Initialize( mid_node, flywheel )
system.Add( rigid_link )





mesh_vis = fea.ChVisualizationFEAmesh( mesh )
mesh_vis.SetFEMglyphType( fea.ChVisualizationFEAmesh.GLYPH_NODE_DOT_POS )
mesh_vis.SetFEMdataType ( fea.ChVisualizationFEAmesh.PLOT_NODE_SPEED_NORM )
mesh_vis.SetColorscaleMinMax( 0, 20 )
mesh_vis.SetSmoothFaces( True )
mesh.AddAsset( mesh_vis )




app = chronoirr.ChIrrApp( system, "Jeffcott rotor – IGA shaft", chronoirr.dimension2du(1280,720) )
app.AddTypicalSky()
app.AddTypicalCamera( chronoirr.vector3df( 0.2, 0.3, 1.1 ) )
app.AddTypicalLights()

app.AssetBindAll()
app.AssetUpdateAll()


system.SetStep( 1e-4 )
system.SetSolverType( chrono.ChSolver.Type_BARZILAIBORWEIN )
system.SetTimestepperType( chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED )




while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()