# Optimize.py
# Created:  Feb 2016, M. Vegh
# Modified: Aug 2017, E. Botero
#           Aug 2018, T. MacDonald

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units, Data
import numpy as np
import Vehicles
import Analyses
import Missions
import Procedure
import Plot_Mission
import matplotlib.pyplot as plt
from SUAVE.Optimization import Nexus, carpet_plot
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup

# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def main():
    
    problem = setup()
    output  = scipy_setup.SciPy_Solve(problem)
    
    print output
    
    ## Uncomment to view contours of the design space
    variable_sweep(problem)
     
    
    ## Uncomment these lines when you want to start an optimization problem from a different initial guess
    #inputs                                   = [1.28, 1.38]
    #scaling                                  = problem.optimization_problem.inputs[:,3] #have to rescale inputs to start problem from here
    #scaled_inputs                            = np.multiply(inputs,scaling)
    #problem.optimization_problem.inputs[:,1] = scaled_inputs
    #output = scipy_setup.SciPy_Solve(problem,solver='SLSQP')
    
    problem.translate(output)

    Plot_Mission.plot_mission(problem.results.mission)
    
    return


# ----------------------------------------------------------------------        
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------  

def setup():

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------

    #   [ tag                            , initial, (lb,ub)             , scaling , units ]
    problem.inputs = np.array([
        [ 'wing_aspect_ratio'      ,  2.7935   , (   1. ,   7.   ) ,   2.7935 , Units.less],
        [ 'wing_area'              ,  68.172    , (   50.  ,    80.   ) ,   68.172  , Units.meter**2],
        [ 'cruise_airspeed'        ,  450.    , (   40.  ,    120.   ) ,   70.  , Units.knots],
        [ 'climb1_airspeed'        , 268.25   , (   100.0  , 200.0      ) ,  268.25   , Units.knots],
        [ 'climb2_airspeed'        ,  326.56   , ( 300.0    ,  400.0     ) ,   326.56  , Units.knots],
        [ 'descent1_airspeed'      ,  440.0  , ( 400.0    ,  500.0     ) ,  440.0   , Units.knots],
        [ 'descent2_airspeed'      ,   250.0   , ( 200.0    ,  300.0     ) ,  250.0   , Units.knots],
        [ 'climb1_altitude'        ,   3.  , (  1.   ,  5.     ) ,  3.   , Units.km],
        [ 'climb2_altitude'        ,   11.28   , (   7.  ,  15.     ) , 11.28    , Units.km],
        [ 'descent1_altitude'      ,  3.657    , (  2.   ,  7.     ) ,   3.657  , Units.km],
        [ 'climb1_rate'        ,    15.24  , (   5.  ,  20.     ) ,  15.24   , Units['m/s']],
        [ 'climb2_rate'        ,    9.14  , (  3.   ,   10.    ) ,   9.14  , Units['m/s']],
        [ 'descent1_rate'      ,   11.68   , (  3.   ,   15.    ) ,  11.68   , Units['m/s']],
        [ 'descent2_rate'      ,    7.62  , (  2.   ,   10.    ) ,  7.62   , Units['m/s']],
        [ 'voltage'      ,   461.   , (  100.   ,   700.    ) ,  461.   , Units.watts],
    ])

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # throw an error if the user isn't specific about wildcards
    # [ tag, scaling, units ]
    problem.objective = np.array([
        [ 'range', -10, Units.nautical_miles ]
    ])
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
        #[ 'energy_constraint', '=', 0.0, 1.0, Units.less],
        [ 'battery_mass'     , '>', 0.0, 1.0, Units.kg  ],       
        #[ 'CL'               , '>', 0.0, 1.0, Units.less],
        [ 'Throttle_min'     , '>', 0.0, 1.0, Units.less],
        [ 'Throttle_max'     , '>', 0.0, 1.0, Units.less],
        [ 'HL_rpm'           , '>=', 0.0, 100, Units['rpm']],
        [ 'cruise_rpm'       , '>=', 0.0, 100, Units['rpm']],
    ])
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        [ 'wing_aspect_ratio'          ,    'vehicle_configurations.*.wings.main_wing.aspect_ratio'   ],
        [ 'wing_area'                  ,     'vehicle_configurations.*.wings.main_wing.areas.reference'         ],
        [ 'cruise_airspeed'            ,   'missions.mission.segments.cruise.air_speed'              ],
        [ 'climb1_airspeed'            ,   'missions.mission.segments.climb_1.air_speed'             ],
        [ 'climb2_airspeed'            ,   'missions.mission.segments.climb_2.air_speed'              ],
        [ 'descent1_airspeed'            ,  'missions.mission.segments.descent_1.air_speed'               ],
        [ 'descent2_airspeed'            ,  'missions.mission.segments.descent_2.air_speed'               ],
        [ 'climb1_altitude'            ,   'missions.mission.segments.climb_1.altitude_end'              ],
        [ 'climb2_altitude'            ,   'missions.mission.segments.climb_2.altitude_end'              ],
        [ 'descent1_altitude'            , 'missions.mission.segments.descent_1.altitude_end'                ],
        [ 'climb1_rate'            ,   'missions.mission.segments.climb_1.climb_rate'              ],
        [ 'climb2_rate'            ,   'missions.mission.segments.climb_2.climb_rate'              ],
        [ 'descent1_rate'            ,  'missions.mission.segments.descent_1.descent_rate'               ],
        [ 'descent2_rate'            ,  'missions.mission.segments.descent_2.descent_rate'               ],
        [ 'voltage'            ,  'vehicle_configurations.*.propulsors.propulsor.voltage'               ],
        [ 'range'            ,                 ],
        [ 'battery_mass'            , 'vehicle_configurations.base.propulsors.network.battery.mass_properties.mass'                ],
        [ 'Throttle_min'            ,   'summary.throttle_min'              ],
        [ 'Throttle_max'            ,   'summary.throttle_max'              ],
        [ 'HL_rpm'            ,  'state.conditions.propulsion.rpm_lift'           ],
        [ 'cruise_rpm'            ,  'state.conditions.propulsion.rpm_forward'             ],
    ]     
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.setup()
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = Analyses.setup(nexus.vehicle_configurations)
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Missions.setup(nexus.analyses,nexus.vehicle_configurations)
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.setup()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()    
    nexus.total_number_of_iterations = 0
    
    return nexus

def variable_sweep(problem):    
    number_of_points = 5
    outputs     = carpet_plot(problem, number_of_points, 0, 0)  #run carpet plot, suppressing default plots
    inputs      = outputs.inputs
    objective   = outputs.objective
    constraints = outputs.constraint_val
    plt.figure(0)
    CS   = plt.contourf(inputs[0,:],inputs[1,:], objective, 20, linewidths=2)
    cbar = plt.colorbar(CS)
    
    cbar.ax.set_ylabel('fuel burn (kg)')
    CS_const = plt.contour(inputs[0,:],inputs[1,:], constraints[0,:,:])
    plt.clabel(CS_const, inline=1, fontsize=10)
    cbar = plt.colorbar(CS_const)
    cbar.ax.set_ylabel('fuel margin')
    
    plt.xlabel('Wing Area (m^2)')
    plt.ylabel('Cruise Altitude (km)')
    
    plt.legend(loc='upper left')  
    plt.show(block=True)    
    
    return



if __name__ == '__main__':
    main()