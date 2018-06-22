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
import Procedure2
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
        [ 'wing_aspect_ratio'      ,  2.7935   , (   1. ,   7.   ) ,   2.7935. , Units.less],
        [ 'wing_area'              ,  68.172    , (   50.  ,    80.   ) ,   68.172  , Units.meter**2],
        [ 'cruise_airspeed'        ,  70.    , (   40.  ,    120.   ) ,   70.  , Units.knots],
        [ 'climb1_airspeed'        ,      , (     ,       ) ,     , Units.knots],
        [ 'climb2_airspeed'        ,      , (     ,       ) ,     , Units.knots],
        [ 'descent1_airspeed'      ,      , (     ,       ) ,     , Units.knots],
        [ 'descent2_airspeed'      ,      , (     ,       ) ,     , Units.knots],
        [ 'cruise_altitude'        ,      , (     ,       ) ,     , Units.km],
        [ 'climb1_altitude'        ,      , (     ,       ) ,     , Units.km],
        [ 'descent1_altitude'      ,      , (     ,       ) ,     , Units.km],
        [ 'climb1_rate'        ,      , (     ,       ) ,     , Units['m/s']],
        [ 'climb2_rate'        ,      , (     ,       ) ,     , Units['m/s']],
        [ 'descent1_rate'      ,      , (     ,       ) ,     , Units['m/s']],
        [ 'descent2_rate'      ,      , (     ,       ) ,     , Units['m/s']],
        [ 'HL_rpm'           ,      , (     ,       ) ,     , Units['rpm']],
        [ 'cruise_rpm'       ,      , (     ,       ) ,     , Units['rpm']],
        [ 'motor_power'      ,      , (     ,       ) ,     , Units.watts],
    ])

    # -------------------------------------------------------------------
    # Objective
    # -------------------------------------------------------------------

    # throw an error if the user isn't specific about wildcards
    # [ tag, scaling, units ]
    problem.objective = np.array([
        [ 'range', 15, Units.km ]
    ])
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    
    # [ tag, sense, edge, scaling, units ]
    problem.constraints = np.array([
        [ 'design_range_fuel_margin' , '>', 0., 1E-1, Units.less], #fuel margin defined here as fuel 
    ])
    
    # -------------------------------------------------------------------
    #  Aliases
    # -------------------------------------------------------------------
    
    # [ 'alias' , ['data.path1.name','data.path2.name'] ]

    problem.aliases = [
        [ 'wing_aspect_ratio'          ,                 ],
        [ 'wing_area'                  ,                 ],
        [ 'cruise_airspeed'            ,                 ],
        [ 'climb1_airspeed'            ,                 ],
        [ 'climb2_airspeed'            ,                 ],
        [ 'descent1_airspeed'            ,                 ],
        [ 'descent2_airspeed'            ,                 ],
        [ 'cruise_altitude'            ,                 ],
        [ 'climb1_altitude'            ,                 ],
        [ 'descent1_altitude'            ,                 ],
        [ 'climb1_rate'            ,                 ],
        [ 'climb2_rate'            ,                 ],
        [ 'descent1_rate'            ,                 ],
        [ 'descent2_rate'            ,                 ],
        [ 'HL_rpm'            ,                 ],
        [ 'cruise_rpm'            ,                 ],
        [ 'motor_power'            ,                 ],
        [ 'range'            ,                 ],
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
    nexus.missions = Missions.setup(nexus.analyses)
    
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



if __name__ == '__main__':
    main()