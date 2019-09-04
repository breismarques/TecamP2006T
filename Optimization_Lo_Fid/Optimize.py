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
     
    
    ## Uncomment these lines when you want to start an optimization problem from a different initial guess
    #inputs                                   = [1.28, 1.38]
    #scaling                                  = problem.optimization_problem.inputs[:,3] #have to rescale inputs to start problem from here
    #scaled_inputs                            = np.multiply(inputs,scaling)
    #problem.optimization_problem.inputs[:,1] = scaled_inputs
    #output = scipy_setup.SciPy_Solve(problem,solver='SLSQP')
    
    problem.translate(output)

    Plot_Mission.plot_mission(problem)
    
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
        [ 'wing_span'      ,  9.0   , (   5.0 ,   20.   ) ,   9.0 , Units.meter],
        [ 'wing_root_chord'  ,  1.0    , (   1.  ,    6.   ) ,   1.0  , Units.meter],
        [ 'cruise_distance'        ,  150.    , (   50.  ,    1000.   ) ,   150.  , Units.nautical_miles],
        [ 'cruise_airspeed'        ,  200.   , (   90.  ,    300.   ) ,   200.  , Units['m/s']],
        [ 'climb1_airspeed'        , 125.0   , (   50.0  ,  250.0      ) ,  125.0   , Units['m/s']],
        [ 'climb2_airspeed'        ,  190.0   , ( 70.0    ,  270.0     ) ,   190.0  , Units['m/s']],
        [ 'descent1_airspeed'      ,  180.0  , ( 100.    ,  270.     ) ,  180.0   , Units['m/s']],
        [ 'descent2_airspeed'      ,   145.0   , ( 50.    ,  200.0     ) ,  145.0   , Units['m/s']],
        [ 'climb1_altitude'        ,   3.  , (  1.   ,  5.     ) ,  3.   , Units.km],
        [ 'climb2_altitude'        ,   8.0   , (   7.  ,  15.     ) , 8.0    , Units.km],
        [ 'descent1_altitude'      ,  3.0    , (  2.   ,  7.     ) ,   3.0  , Units.km],
        [ 'climb1_rate'        ,    6.0  , (   3.  ,  17.     ) ,  6.0   , Units['m/s']],
        [ 'climb2_rate'        ,    3.0  , (  2.   ,   12.    ) ,   3.0  , Units['m/s']],
        [ 'descent1_rate'      ,   4.5   , (  3.   ,   15.    ) ,  4.5   , Units['m/s']],
        [ 'descent2_rate'      ,    3.0  , (  2.   ,   12.    ) ,  3.0   , Units['m/s']],
        [ 'voltage'      ,   470.   , (  100.   ,   10000.    ) ,  470.   , Units['volt']],
        [ 'motor_kv_cruise'      ,   181.   , (  10.   ,   500.    ) ,  181.   , Units['rpm/volt']],
        [ 'motor_kv_HL'      ,   91.   , (  5.   ,   300.    ) ,  91.   , Units['rpm/volt']],
        [ 'bat_spec_energy'      ,   6.1  , (  3.   ,   10000.    ) ,  6.1  , Units.Wh/Units.kg],
        [ 'bat_spec_power'      ,   0.833   , (  0.5   ,   1000.    ) ,  0.833   , Units.kW/Units.kg],
        [ 'bat_max_voltage'      ,   60.1   , (  0.5   ,   100000.    ) ,  60.1   , Units['volt']],
        [ 'bat_resistance'      ,   0.0151   , (  0.0001   ,   100.    ) ,  0.0151   , Units['ohm']],
        [ 'payload_draw'      ,   50.1   , (  5.   ,   50000.    ) ,  50.1   , Units.watts],
        [ 'avionics_draw'      ,   50.1   , (  5.   ,   50000.    ) ,  50.1   , Units.watts],
    ])

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # throw an error if the user isn't specific about wildcards
    # [ tag, scaling, units ]
    problem.objective = np.array([
        [ 'range', -540.9079921321364, Units.nautical_miles ]
    ])
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
        #[ 'energy_constraint', '=', 0.0, 1.0, Units.less],
        [ 'battery_energy'     , '>', -0.1, 1.0, Units.Wh  ],
        #[ 'battery_draw'     , '>', -0.1, 1.0, Units.kW  ],
        #[ 'CL'               , '>', 0.0, 1.0, Units.less],
        [ 'Throttle_cruise'     , '>', -0.1, 1.0, Units.less],
        [ 'Throttle_HL'     , '>', -0.1, 1.0, Units.less],
        [ 'Throttle_cruise'     , '<', 1.5, 1.0, Units.less],
        [ 'Throttle_HL'     , '<', 1.5, 1.0, Units.less],
        [ 'HL_rpm'           , '>', -0.1, 1.0, Units['rpm']],
        [ 'cruise_rpm'       , '>', -0.1, 1.0, Units['rpm']],
        [ 'lift_coefficient'       , '>', -0.1, 1.0, Units.less],
        [ 'mach_number'       , '<', 1.5, 1.0, Units.less],
    ])
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        [ 'wing_span'          ,    ['vehicle_configurations.base.wings.main_wing.spans.projected',
                                     'vehicle_configurations.cruise.wings.main_wing.spans.projected'] ],
        [ 'wing_root_chord'                  ,     'vehicle_configurations.*.wings.main_wing.chords.root'         ],
        [ 'cruise_distance'            ,   'missions.mission.segments.cruise.distance'              ],
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
        [ 'motor_kv_cruise'            ,  'vehicle_configurations.*.propulsors.propulsor.motor_forward.speed_constant'               ],
        [ 'motor_kv_HL'            ,  'vehicle_configurations.*.propulsors.propulsor.motor_lift.speed_constant'               ],
        [ 'bat_spec_energy'            ,  'vehicle_configurations.*.propulsors.propulsor.battery.specific_energy'               ],
        [ 'bat_spec_power'            ,  'vehicle_configurations.*.propulsors.propulsor.battery.specific_power'               ],
        [ 'bat_max_voltage'            ,  'vehicle_configurations.*.propulsors.propulsor.battery.max_voltage'               ],
        [ 'bat_resistance'            ,  'vehicle_configurations.*.propulsors.propulsor.battery.resistance'               ],
        [ 'payload_draw'            ,  'vehicle_configurations.*.propulsors.propulsor.payload.power_draw'               ],
        [ 'avionics_draw'            ,  'vehicle_configurations.*.propulsors.propulsor.avionics.power_draw'               ],
        [ 'range'            ,    'summary.total_range'            ],
        [ 'battery_energy'            , 'summary.battery_energy_all_segments'                ],
        [ 'battery_draw'            , 'summary.battery_charging_power_all_segments'                ],
        [ 'Throttle_cruise'            ,   'summary.throttle_all_segments'              ],
        [ 'Throttle_HL'            ,   'summary.lift_throttle_all_segments'              ],
        [ 'HL_rpm'            ,  'summary.rpm_lift_all_segments'           ],
        [ 'cruise_rpm'            ,  'summary.rpm_forward_all_segments'             ],
        [ 'lift_coefficient'            ,  'summary.lift_coefficient_all_segments'             ],
        [ 'mach_number'            ,  'summary.mach_number_all_segments'             ],
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
    #nexus.summary = Data()  
    
    nexus.total_number_of_iterations = 0
    
    return nexus


if __name__ == '__main__':
    main()