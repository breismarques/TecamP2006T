# Procedure2.py

# Python Imports
import numpy as np
import pylab as plt
from subprocess import call
import time
#import matlab.engine
import statsmodels.formula.api as smf
import pandas

# SUAVE Imports

import SUAVE
from SUAVE.Core import Units, Data
import numpy as np
import matplotlib.pyplot as plt
from SUAVE.Optimization import Nexus, carpet_plot
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
from SUAVE.Input_Output.OpenVSP import vspaero




# Files Imports

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def main():
    
    vel_sound=340
    rho=1.2250
    
    AoA=np.array([7.0, 0.4, 5.0, 15.0, 40.0,
         6.0, 9.0, 5.0, 0.0, 21.0]) * Units.deg
    
    #AoA=np.transpose(AoA)    
    
    mach=np.array([0.44079532457429516, 0.3, 0.5, 0.21, 0.1, 
          0.81195286433823222, 0.511229581250084, 0.50823848545971062, 0.50240914350715393, 0.49956803451753434])
    
    #mach=np.transpose(mach)
    
    iters=3
    
    rpm_forward=np.array([78.0, 0.001, 7570.218883021749, 20.0, 10.0, 
                 450.0, 350.0, -2000.0, 300.0, 70.0]) * Units['rpm']
    
    #rpm_forward=np.transpose(rpm_forward)
    
    rpm_lift=np.array([46.0, 3784.575248141249, 4000.0, 30.0, -100.0, 
              1500.0, 350.0, 750.0, 1200.0, 65.0]) * Units['rpm']
    
    #rpm_lift=np.transpose(rpm_lift)
    
    engines_number_tot=14
    
    Cp_lift=np.array([10.0, -1000.00000, 0.0, 200.0, 10.0, 
             -10.0, 44.45010057682831, 1000.0, 350.0, -39.573102476674528])
    
    #Cp_lift=np.transpose(Cp_lift)
    
    Cp_forward=np.array([60.4271376019077016, -146.57880339094905, 248.15478376178456, 20.13225978032551, -41.159087723621411,
                600.0, 55.065564919157, 122.44107478164513, 1000.0, -495.62502937595718])
    
    #Cp_forward=np.transpose(Cp_forward)
    
    Ct_lift=np.array([0.001, 100.0, -1000.0, 44.0, 20.0,
             80.0, 150.0, 900.0, 90.0, 550.0])
    
    #Ct_lift=np.transpose(Ct_lift)
    
    Ct_forward=np.array([10.0, 0.001, 200.0, 15.0, 750.0,
                500.0, 1000.0, 650.0, 90.0, 800.0])
    
    #Ct_forward=np.transpose(Ct_forward)
    
    with open('inputs.txt', 'w') as f:
        for item in AoA:
            f.write("%f\n" % item)
        f.write("\n")
        for item in mach:
            f.write("%f\n" % item)
        f.write("\n")
        for item in rpm_forward:
            f.write("%f\n" % item)
        f.write("\n")
        for item in rpm_lift:
            f.write("%f\n" % item)
        f.write("\n")
        for item in Cp_lift:
            f.write("%f\n" % item)
        f.write("\n")
        for item in Cp_forward:
            f.write("%f\n" % item)
        f.write("\n")
        for item in Ct_lift:
            f.write("%f\n" % item)
        f.write("\n")
        for item in Ct_forward:
            f.write("%f\n" % item)
        f.write("\n")
            
    f.close()
    
    i=0
    data_len = len(AoA)
    CL = np.zeros(data_len)
    CD = np.zeros(data_len)

    
    while i < len(AoA):
    
        CL[i], CD[i] = vspaero(vel_sound,"climb.vsp3", rho, AoA[i], mach[i], iters, rpm_forward[i], rpm_lift[i], engines_number_tot, Cp_lift[i], Cp_forward[i], Ct_lift[i], Ct_forward[i])
        print "CL= %f" % CL[i]
        i=i+1
    
    
    with open('CL_results.txt', 'w') as f:
        for item in CL:
            f.write("%f\n" % item)
    
    f.close()
    
    df=pandas.DataFrame({'AoA': AoA, 'mach': mach, 'rpm_forward': rpm_forward, 'rpm_lift': rpm_lift, 'Cp_lift': Cp_lift, 'Cp_forward': Cp_forward, 'Ct_lift': Ct_lift, 'Ct_forward': Ct_forward, 'CL': CL})
    
    print df
    
    mod = smf.ols(formula='CL~mach+np.power(AoA,2)+np.power(rpm_forward,2)+np.power(rpm_lift,2)+AoA*mach+rpm_forward*Cp_forward+rpm_forward*Ct_forward+rpm_lift*Cp_lift+AoA*rpm_forward+AoA*rpm_lift', data=df)
    
    #mod = smf.ols(formula='CL~mach+np.power(AoA,2)+np.power(rpm_forward,2)+np.power(rpm_lift,2)+AoA*mach+rpm_forward*Cp_forward+rpm_forward*Ct_forward+rpm_lift*Cp_lift+mach+AoA*rpm_forward+AoA*rpm_lift+mach*Cp_forward+mach*Cp_lift+mach*Ct_forward+mach*Ct_lift', data=df)
    
    #mod = smf.ols(formula='CL~mach+np.power(AoA,2)+AoA*Ct_forward+AoA*rpm_lift+rpm_forward*Ct_forward+rpm_lift*Cp_lift+mach*rpm_forward+mach*rpm_lift', data=df)
    
    res = mod.fit()
    
    print(res.summary())
    
    i=0
    
    CL_predict = np.zeros(data_len)
    
    print res.fittedvalues
    
    print res.params.values
    
    #while i<len(AoA):
        
    #    CL_predict[i]=mach+np.power(AoA,2)+AoA*mach+rpm_forward*Cp_forward+rpm_forward*Ct_forward+rpm_lift*Cp_lift+mach*rpm_forward+mach*rpm_lift+np.power(rpm_forward,2)+np.power(rpm_lift,2)+AoA*rpm_forward+AoA*rpm_lift
    
    
    #eng = matlab.engine.start_matlab('-desktop')
    
    #coeff = eng.linearregression2(AoA,mach,rpm_forward,rpm_lift,Cp_lift,Cp_forward,Ct_lift,Ct_forward,inviscid_lift)
    
    #print coeff
    
    AoA=np.array([30.0, 15.0, 40.0, 5.0]) * Units.deg
    mach=np.array([0.25, 0.4, 0.6, 0.1])
    rpm_forward=np.array([3.0, 100.0, 300.0, 50.0]) * Units['rpm']
    rpm_lift=np.array([30.0, 5.0, 100.0, 600.0]) * Units['rpm']
    Cp_lift=np.array([20.0, 10.0, 400.0, 600.0])
    Cp_forward=np.array([100.0, 30.0, 200.0, 500.0])
    Ct_lift=np.array([50.0, 500.0, 200.0, 300.0])
    Ct_forward=np.array([150.0, 50.0, 400.0, 500.0])
    
    df=pandas.DataFrame({'AoA': AoA, 'mach': mach, 'rpm_forward': rpm_forward, 'rpm_lift': rpm_lift, 'Cp_lift': Cp_lift, 'Cp_forward': Cp_forward, 'Ct_lift': Ct_lift, 'Ct_forward': Ct_forward})
    
    result_predict=res.predict(df)
    
    i=0
    data_len = len(AoA)
    CL = np.zeros(data_len)
    CD = np.zeros(data_len)
    
    while i < len(AoA):
    
        CL[i], CD[i] = vspaero(vel_sound,"climb.vsp3", rho, AoA[i], mach[i], iters, rpm_forward[i], rpm_lift[i], engines_number_tot, Cp_lift[i], Cp_forward[i], Ct_lift[i], Ct_forward[i])
        i=i+1
        
    print "Predict VALUES"
    
    for elem in result_predict:
        print elem
        
    print "VSPAERO VALUES"
    
    for elem in CL:
        print elem
    
    
    return

if __name__ == '__main__':
    main()
    
    