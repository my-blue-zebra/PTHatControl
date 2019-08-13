# -*- coding: utf-8 -*-
"""
Created on Mon Aug 12 20:11:52 2019
Command generator
@author: micha
"""
#from PTHmodule import *



import numpy as np
def CalcSteps(AngleOld, AngleNew, StepsPerRevolution, Gearing):
    AngleDiff = AngleNew-AngleOld
    
    Dirs = np.zeros([3])    
    AnglePerStep = 2*np.pi / (StepsPerRevolution * Gearing)
    
    for i in range(3):
        if AngleDiff[i] >= 0:
            Dirs[i] = int(1);
        else:
            Dirs[i] = int(0);
            AngleDiff[i] = - AngleDiff[i]
    Steps = (AngleDiff//AnglePerStep)
    return Steps, Dirs
    
def CalcFreq(Steps, Time):
    Freqs = Steps/Time
    return Freqs

def MakeCommand(PT, Steps, Direction, Freq, RampS, RampF, SPR, Gear):
    CID = "01"
    CommX = "B" + CID + "CX"
    CommY = "B" + CID + "CY"
    CommZ = "B" + CID + "CZ"
    
    Comm = [CommX, CommY, CommZ]
    
    for i in range(3):
        sFreq  = "{0:010.3f}".format(Freq[i])
        sPulse = "{0:010d}".format(int(Steps[i]))
        sDir   = "{0:1d}".format(int(Direction[i]))
        sRamp = ""
        if RampS:
            sRamp += "1"
        else:
            sRamp += "0"
        if RampF:
            sRamp += "1"
        else:
            sRamp += "0"
        #print(sRamp)
        Comm[i] += sFreq + sPulse + sDir + sRamp + "000100000*"
        
    for Axis in range(3):
        if Axis == 0:
            axis = PT.GetXaxis
        elif Axis == 1:
            axis = PT.GetYaxis
        elif Axis == 2:
            axis = PT.GetZaxis
        axis.StepsPerUnit = SPR * Gear
        ch = axis.__channel
        sDir   = "{0:1d}".format(int(Direction[i]))
        CW = sDir + "000000"
        axis.__pthat.SetAxisMove(ch, Steps[Axis], Freq[Axis], CW)
    return Comm

def StartHat(debug = False):
    PT = PTHAT()
    PT.DEBUG = debug
    S = PT.GetVersion
    print(S)
    return PT
    

def SendPTHCommand(PT, Steps, Direction, Freq, RampStime, RampFtime, SPR, ):
    for Axis in range(3):
        if Axis == 0:
            axis = PT.GetXaxis
        elif Axis == 1:
            axis = PT.GetYaxis
        elif Axis == 2:
            axis = PT.GetZaxis

        
            
    