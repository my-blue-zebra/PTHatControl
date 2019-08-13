###############################################
# Pulse Train Hat serial port handler module
#
#(c) W.K.Todd  22/12/2018 - for V5.3
#
################################################
import time
import serial
import math

class AUX(object):
    '''PTHAT AUX object to handle aux output lines'''
    
    __OP = False
    

    def __init__(self, channel, pthat):
        #save pthat object for callbacks
        self.__channel = channel
        self.Name = "AUX" + channel  #default name
        self.__pthat = pthat #refernce the creator object

        #print(channel , pthat, self) #debug

    def __del__(self):
        #log out of pthat
        pass
    def __repr(self):
        return 'Pulse Train Hat AUX Object:' + self.Name
    
    
    @property
    def GetChannel(self):
        return self.__channel

    @property
    def GetAux(self):
        return self.__OP
    
    
    def SetAux(self, OP=True):
        self.__OP = OP
        self.__pthat.SetAUX(self,OP)
        
#-----------------------------------------------------


class ADC(object):
    '''PTHAT ADC object to handle ADC reads interactions'''

    V = 0 #current ADC reading - long integer 0-4096  12bit

    def __init__(self, channel, pthat):
        #save pthat object for callbacks
        self.__channel = channel
        self.Name = "ADC" + channel  #default name
        self.__pthat = pthat #refernce the creator object

        #print(channel , pthat, self) #debug

    def __del__(self):
        #log out of pthat
        pass
    def __repr__(self):
        return 'Pulse Train Hat ADC{} Object: {}'.format(self.__channel, self.Name)
    
    @property
    def GetChannel(self):
        return self.__channel

    @property
    def Value(self):
        self.__pthat.ReadADC(self) #ask hat for response
        
        return self.V

#----------------------------------------------------
    
class Axis(object):
    '''PTHAT Axis object to handle axis interactions:
      variables
      Name="" #user name for axis channel e.g. Wrist, Elbow, Shoulder
    __channel = "" #channel used for PTHAT  X,Y,Z or E readonly
    Units anything you like (e.g. Revs, metres ,mm, inches, feet )
    Speed in Units per Minute
    AccelerationTime in seconds (float)
    __pthat # reference back to PTHAT object
    '''

    __limitenabled = False
    Clockwise=True
    StepsPerUnit =1.0 #steps per unit  (e.g 1045.67/mm , 12800/Revolution, or 25400/inch)
    Unit ="" #user unit variable (e.g. Revs, mm, Inch)
    StepsToDo = 0 #number of steps (integer) to do in Move
    Freq=0 #calculated frequency of Move
    AccelerationTime = 0.0 #seconds (float)
    RampSpeed=""
    UpRampEnable = True
    DownRampEnable= True
    ADCLink = 0 #linked to ADC 1,2 or 0 off
    MotorEnable = True
    IsRunning = False
    AtLimit = False
    Paused = False
    StepsDone = 0 #number of steps so far (current position)
    ClockwiseIsNegative = False #reverse motor direction
    
    def __init__(self, channel, pthat):
        #save pthat object for callbacks
        self.__channel = channel
        self.Name = channel + "axis" #default name
        self.__pthat = pthat #refernce the creator object

        #print(channel , pthat, self) #debug

    def __del__(self):
        #log out of pthat

        pass
    def __repr__(self):
        return 'Pulse Train Hat {} Axis Object: {}'.format( Self.__channel , self.Name )
    
    @property
    def GetChannel(self):
            return self.__channel

    @property
    def GetCurrentPosition(self, Request = False):
        if Request : self.__pthat.GetAxisSteps(self)
        return self.StepsDone / self.StepsPerUnit
 
    @property
    def GetLimitEnable(self):
            return self.__limitenabled  

    def SetMove(self, Units, UnitsPerMin, Clockwise =True): #speed in units per minute
            #calculate steps
            self.StepsToDo = abs(int(Units * self.StepsPerUnit))
            #calculate frequency
            self.Freq = UnitsPerMin * self.StepsPerUnit / 60
            if Units < 0:
                self.Clockwise = False != self.ClockwiseIsNegative #xor with CW=Neg
            else:
                self.Clockwise = Clockwise != self.ClockwiseIsNegative
            
             #Completion Word  with ramps #debug this needs work
            
            if self.Clockwise:
                CW="0"
            else:
                CW ="1"

            if self.UpRampEnable:
                CW +="1"
            else:
                CW +="0"
                
            if self.DownRampEnable:
                CW +="1"
            else:
                CW +="0"
            
            if self.RampSpeed =="" : 
                CW += self.RampString() # add acceleration time ramp string
            else:
                CW += self.RampSpeed #for backward compatibility only.
                
            
            CW += str(self.ADCLink) # 0,1,2

            if self.MotorEnable:
                CW += "1"
            else:
                CW += "0"
            
            #call PTHAT
            self.__pthat.SetAxisMove(self.__channel,self.StepsToDo,self.Freq ,CW)
            
    def SetAutoReverse(self, Units):
        #calculate steps
        self.StepsToDo = abs(int(Units * self.StepsPerUnit))        
        #call PTHAT
        self.__pthat.SetAxisAutoReverse(self.__channel,self.StepsToDo)
   
    def LimitEnable(self, enable=True):
        self.AtLimit = False
        self.__limitenabled = enable
        self.__pthat.SetAxisLimit(self, enable) 

    def Start(self):
        self.__pthat.StartAxis(self)            
        self.IsRunning = True    
    def Stop(self):
        self.__pthat.StopAxis(self)

    def Pause(self):
        self.__pthat.PauseAxis(self)

    def SpeedChange(self, UnitsPerMin):
        #instant speed change
        self.Freq = UnitsPerMin * self.StepsPerUnit / 60
        self.__pthat.AxisSpeedChange(self.__channel, self.Freq)

    def RampString(self):
        #takes accelleration time (t mS) and returns Ramp string
        T = int(math.sqrt(self.AccelerationTime * 1000))
        D = T+1
        return '{:0=3}'.format(D) +'{:0=3}'.format(T)
#---------------------------------------------------------------



class PTHAT(object):
    '''PTHAT object to handle PTHAT interactions'''
    
    __SP = None #object() #place holder for serial port object for PTHAT
    SID =0 #instruction count sent id integer = 0-99
    RID = 0 #instruction count returned
    CID = 0 #instructions completed
    __IorB = "I" #immediate or buffer 'B'
    RS = "" #response string from hat
    version =""
    Limit = False
    Estop = False
    DEBUG = False #enable prints debug statements
    PTOs = {}  #dictionary to hold pt objects like ADCs, AUXs  Axes etc.
    Buffering = False
    BufferEmpty = True
    BufferCount = 0
    
    def __init__(self):
        #print("PTHAT created")

        #create my serial port object and open it
        self.__SP = self.init_serial()
        
        #reset PTHat
        self.sendtohat("N")
        
        
    def __del__(self):
        #send all stop to hat here
        self.sendtohat("N")
        self.__SP.close()
        
    def __repr__(self):
        return 'Pulse Train Hat Object: {}'.format(self.version )
    

#### PTHAT read only properties
        
    @property   
    def Pollport(self):
        #C=0     
        response = None
        w = self.__SP.in_waiting 
        if  w:
                sbyte = self.__SP.read(w) #read serial buffer
                self.RS += sbyte.decode() #convert bytes to string
                C = self.RS.rfind("*")
                response = self.RS[0:C].split("*") #create list of responses
                self.RS = self.RS[C:] # buffer incomplete reponses
                #Parse response to set ADCs, Version etc.
                self.parse_responses(response)
        #return response #return list
        Flag =0       
        if self.Estop:
            Flag += 8
        if self.Buffering and self.BufferEmpty:
            Flag += 4
        if self.Limit:
            Flag += 2
        if w:
            Flag += 1

        return Flag
    
    @property
    def GetVersion(self):
        #send version string =I00FW* (sendto hat takes care if 'I00' bit)
        if self.version == "":
            #if version is empty then wait for version to be assigned by pollport
            self.sendtohat("FW")
                
            while self.version == "":
                x = self.Pollport
            
        return self.version
    
    @property
    def GetXaxis(self):
        return self._getaxis("X")
    
    @property
    def GetYaxis(self):
        return self._getaxis("Y")
    
    @property
    def GetZaxis(self):
        return self._getaxis("Z")
    
    @property
    def GetEaxis(self):
        return self._getaxis("E")
    
    @property
    def GetADC1(self):
        return self._getadc("1")
    
    @property
    def GetADC2(self):               
        return self._getadc("2")

    @property
    def GetAUX1(self):
        return self._getaux("1")
    
    @property
    def GetAUX2(self):
        return self._getaux("2")

    @property
    def GetAUX3(self):
        return self._getaux("3")


############ Methods ##########################
    
    def BufferEnable(self, enable=True):
        #sets up buffer and starts buffering all new commands
        if enable:
            self.sendtohat("H") #initialise buffer
            self.__IorB = "B" #make all following commands buffered
            
        else:
            self.__IorB = "I"

        self.Buffering = enable
        self.BufferEmpty = True
        self.BufferCount =0
            
    def BufferStart(self):
        if self.Buffering: #cant start buffer before initialisation
            self.sendtohat("Z")

    def GetAxisSteps(self, Axis):
        self.sendtohat(Axis.GetChannel + "P")    
    
    def _getadc(self,ch):
        if not "AD" + ch in self.PTOs:
            self.PTOs["AD" + ch] = ADC(ch, self) #add to pto dictionary
        return self.PTOs["AD" + ch]

    def _getaux(self,ch):
        if not "U" +  ch in self.PTOs:
            self.PTOs["U" + ch] = AUX(ch, self) #add to pto dictionary
        return self.PTOs["U" + ch]
    
    def _getaxis(self,ch):
        if not ch in self.PTOs:
            self.PTOs[ch] = Axis(ch, self) #add to pto dictionary
        return self.PTOs[ch]
    
    def SetAUX(self, AUX, enable):
        if enable:
            F="1"
        else:
            F="0"
        self.sendtohat("A"+ AUX.GetChannel +F)
    
    def ReadADC(self, ADC):
        self.sendtohat("D"+ ADC.GetChannel)   
    
    def StartAxis(self, Axis):
        #Start single axis X,Y,Z, or All A
        Axis.IsRunning = True
        self.sendtohat("S"+ Axis.GetChannel)


    def StopAxis(self, Axis):
        #Start single axis X,Y,Z, or All A
        self.sendtohat("T"+ Axis.GetChannel)

    def StartAll(self):
        #start all axes
        self.sendtohat("SA")
        self.__setrunflags()
        
    def StopAll(self):
        #start all axes
        self.sendtohat("TA")
        self.__setrunflags(False)       
        
    def PauseAll(self):
        #stop all axes
        self.sendtohat("PA1111")

    def PauseAxis(self, Axis):
        #Start single axis X,Y,Z, or All A
        self.sendtohat("P"+ Axis.GetChannel + "1111")

    def SetAxisLimit(self, Axis ,enable):
        #set axis limit enable
        #I00KX1*
        if enable:
            F="1"
        else:
            F="0"
        self.sendtohat("K"+ Axis.GetChannel +F)

        self.Limit = False
        #for ch in 'XYZE':
        #    try:
        #        self.Limit = self.Limit or self.PTOs[ch].AtLimit 
        #    except:
        #       pass
            

    def SetAxisMove(self, ch,steps,freq,CW):
        #create formatted string and pass to serial
        self.Limit = False #clear limit flag
        command = "C"
        command += ch
        F= '{:0=13f}'.format(freq)
        command += F[0:10]
        S = '{:0>13}'.format(steps)
        command += S[3:]
        command += CW 
        self.sendtohat(command)
        if self.Buffering:
            self.BufferEmpty = False

    def SetAxisAutoReverse(self,ch, steps):
        command = "B"
        command += ch
        S = '{:0>13}'.format(steps)
        command += S[3:]
        self.sendtohat(command)
        
    def AxisSpeedChange(self,ch, freq):
        #change speed now
        F= '{:0=13f}'.format(freq)
        self.sendtohat("Q" + ch + F[0:10])

    def ToggleMotorEnable(self):
        #invert motor enable outputs
        self.sendtohat("HT")
        
    def Reset(self):
        self.sendtohat("N")
        self.__IorB = "I"
        self.__setrunflags(False)
        
#### Private object methods ########################
        
#initialise serial port for PTHat
#
#Returns: serial port object
#
    def init_serial(self):
        try:
            SP = serial.Serial(
                    port='/dev/serial0',
                    baudrate = 115200,
                    bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE,
                    stopbits = serial.STOPBITS_ONE,
                    xonxoff = False,
                    rtscts = False,
                    write_timeout = 2,
                    timeout = 2
                    ) 
            return(SP)
        except Exception as e:
            #error handler 
            print('init serial - Error opening serial port /n/l %s' % e)
            return(False)

    
    def sendtohat(self,command):  #make private after debugging __sendtohat
        #send command to serial port
        if command == "N":
            #reset special N*
            sth = "N*"
            self.SID = 0

        elif command in "ZH":
            #do buffer commands
            sth = command + "0000*"
                    
        else:
            sid = '{:0>3}'.format(str(self.SID))[1:]
            sth = self.__IorB + sid + command + "*"
            self.SID +=1 #increment counter
            if self.SID >=100: self.SID = 0
             
        self.__SP.write(bytes(sth, 'utf-8'))
        time.sleep(0.012 * len(command))
        if self.DEBUG: print("sendtohat>>>",sth) #debug
        if self.Buffering:
            self.BufferCount +=1
        

    def parse_responses(self,response_list):
        #search list for useful replies
        for res in response_list:
            
            if res !="":
                if self.DEBUG:print("parser>>>" + res) #debug

                #response is a reply so test for ...
                if res[0] == "D":
                    # axis paused
                    self.PTOs[res[5]].Paused = True
                    #self.PTOs[res[5]].IsRunning = False
                    
                if res[0] == "R":
                    
                    if res[2] in "ZH":
                        #set buffering true or false if res ==H
                        self.Buffering = res[2] == "H"
                        
                    elif res[1] in "IB":
                        # self.RID = int(res[2:4])

                         self.RID = self.__extractnum(res)
                     
                #response is an adc
                elif res[0:2] == "AD": #'AD1' or 'AD2'
                        #test ADC
                        try:
                            self.PTOs["AD" + res[2]].V= int(res[5:14])
                        except:
                            pass

                #response is a position 
                elif res[0:2] in "XPYPZPEP":
                        ch = res[0]
                        CW = res[2] == "0"
                        pos = int(res[3:14])
                        #print(ch,CW,pos)
                        self.PTOs[ch].StepsDone = pos
                                             
                #response is a completion code        
                elif res[0] == "C":
                        if res[1] in "IB":
                            self.CID = self.__extractnum(res)

                            
                        if res[4:6] == "FW":  #RI00FW
                            #next response will be a firmware version string
                            self.version = response_list[response_list.index(res) -1]
                            #print("parser>>",self.version)
                    
                        if res[4] == "S" and res[1] == "I":
                            # for immediate I commands axis has completed move and is stopped
                            if res[5] == "A" :
                                # all axes stopped
                                self.__setrunflags(False)

                            else:
                                self.PTOs[res[5]].IsRunning = False

                        if res[4] == "S" and res[1] == "B":
                            # buffered command completed
                            self.BufferCount -=1
                        elif res[1] == "S":
                            #E-Stop triggered
                            self.Estop=True
                            self.__setrunflags(False)

                elif "Empty" in res:
                    #Buffered commands completed flag all stopped
                    self.__setrunflags(False)
                    self.BufferEmpty = True
                    self.BufferCount = 0
                            
                elif "LTrig" in res:
                    #res[1] in "XYZE":
                    #axis limit triggered
                    #a = res[res.rfind('L') -2]
                    self.PTOs[res[0]].AtLimit = True
                    self.__setrunflags(False)
                    self.Limit = True

                elif "ESTrig" in res:
                    #E-Stop triggered
                    self.Estop=True
                    self.__setrunflags(False)
                               
    def __setrunflags(self,F=True):
        for ch in "XYZE":
            try:
                self.PTOs[ch].IsRunning = F
            except:
                pass

    def __extractnum(self, res):
        #print(res)
        c=''
        for s in res:
            if s.isdigit():
                c += s
        return int(c)
    
#----------------------------------------------------------------    
