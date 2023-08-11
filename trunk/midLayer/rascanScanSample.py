import re
import os
import time
import datetime
import yaml
import numpy
from epics import caget, caput, PV
from rascanLib import set_pv_wait, rscnSolver, load_excel_to_dicts, load_dict_to_pvs
from rascanLib import consts

global start_Time, backMsg, eprefix, devErrorFlag, verbose_level
start_Time = time.time()


def handle_update(pvname, value, status, **kwargs):
    global start_Time, backMsg, eprefix, devErrorFlag, verbose_level
    if verbose_level > 3 :
        print(pvname[len(eprefix):], '=', value, ' sts:', status, ' eTm:', round(time.time() - start_Time, 2), 's')
    donothing=value

def handle_rscnRun(pvname, value, status, **kwargs):
    global start_Time, backMsg, eprefix, devErrorFlag, verbose_level
    if verbose_level > 3 :
        print(pvname[len(eprefix):], '=', value, ' sts:', status, ' eTm:', round(time.time() - start_Time, 2), 's')
    print('run is finished')
    backMsg = "Done"
    donothing=value

def handle_deverror(pvname, value, status, **kwargs):
    global start_Time, backMsg, eprefix, devErrorFlag
    devErrorFlag = (value > 0)
    donothing=value

def resetRascan() :
    # try to quit gracefully
    successful_, msg = set_pv_wait(eprefix, ':DEV_CMD', consts.stTerminated, ':EN_INPROG', 0, timout_sec=5, fault_pv='', do_no_status_check=True)
    # if not, then abort
    if not successful_:
        print('goTerminate timeed out, aborting...')
        successful_, msg =set_pv_wait(eprefix, ':ABORT.PROC', 1, ':EN_INPROG', 0, 5, '', False)
        if not successful_ :
            print('Aborting failed... exit with error')
            return False
    return True

devError = 0


dryRun = False
doStdoutLog = False
verbose_level = 0
doAdaptAccel = True



# ------------ Procedure step 1
# ------------ Load Scan Descriptors: Mode configs
# These parameters are used to solve and comile rascan trajectory

#print('\n Loading from '+'ModeConfigs.xlsx'+' ...')
#ModeConfigs = load_excel_to_dicts(file_name='ModeConfigs.xlsx', header_row=2,first_col=2)

filename_ = 'ModeConfigs.yaml'
print('\n Reading '+filename_+' ...')
with open(filename_,'r') as fin :
    ModeConfigs = yaml.load(fin)

# setting up Rascan master control PV's
eprefix = ModeConfigs[0]['EPICS_PREFIX']  # 'WORKSHOP01:RSCN'
PVLineN = PV(eprefix+':LINE_N', callback=handle_update)
PVdevError = PV(eprefix+':DEV_ERROR', callback=handle_deverror)
PVdevState = PV(eprefix+':DEV_STATE', callback=handle_update)
PVlineSubN = PV(eprefix+':LINE_SUBN', callback=handle_update)
PVdevCMD = PV(eprefix+':DEV_CMD', callback=handle_update)
rscnRUN = PV(eprefix+':RUN', callback=handle_rscnRun)


# hardcode some values!
#ModeConfigs[0]['A1_HLM'] = 300
#ModeConfigs[0]['A2_HLM'] = 1

#ModeConfigs[0]['J1_HLM'] = ModeConfigs[0]['A1_HLM']/0.01
#ModeConfigs[0]['J2_HLM'] = ModeConfigs[0]['A2_HLM']/0.01

#ModeConfigs[0]['CS_ACTIVE'] = '4'


#time scaling
timeScaleFactor = 1

filename_ = 'ModeConfigs.dump.yaml'
print('\n dumping to '+filename_+' ...')
with open(filename_,'w+') as stream :
    stream.write(yaml.safe_dump(ModeConfigs))
    stream.close()



# ------------- Reset Rascan ----------------
# When:
# - Need to disengage scanning motors e.g. to change sample mount, change 
# EntryConds: 
# - Ensure scan is not running
# - KEEP sample shutter closed
#
print('\n Make sure Rascan is reset...')
start_Time = time.time()
if not resetRascan() :
    print('Failed to reset Rascan.')
    exit(1)

# TimeToDo:
# - Change sample mount
# - Change scanning geometry/mode 

# ------------- Activate specified CS ----------------
# - Need to use CS motors
# EntryConds: 
# - CS is determined
# - Changes and setups for new CS (mode) on sample and setector stages are done
#
# - Rascan prog is not runing (INPROG=0)

csActive=ModeConfigs[0]['CS_ACTIVE']
print('\n Activating coordinate system ('+csActive+')...')
set_pv_wait(eprefix, ':CS_ACTIVE', csActive, ':CS_ACTIVE:RBV', 4, 3, '', False)
set_pv_wait(eprefix, ':DEV_CMD', consts.stStandby, ':EN_INPROG', 0, 3, '', False)

time.sleep(1)

# TimeToDo:
# - test and validate Excluded Zones
# - Home motors and verify user coordinates (OFFsets of real motors)
# - Verify and setup CS motor user coordinate


# ------------- Start Rascan in specified CS ----------------
# EntryConds: 
# - CS is activated and setup (exclusion zones and kinematic limits are entered)
# - No need to move motors engaged in scan. Move to origin will be handled by Rascan
# - Rascan program is not runing (INPROG=0)

print('\n Starting RASCAN prog in coordinate system '+csActive+'...')
successful_, msg = set_pv_wait(eprefix, ':RUNPROG.PROC', 0, ':EN_INPROG', 1, 5, '', True)
if not successful_ :
    print('Failed:'+ msg)
    exit(1)

time.sleep(0.5)

print('Rascan prog is ready...')

# ------------- Batch setup -----------------------
# Setup PV parameters which apply to (and are constant through) a Batch of Sequences
# This is the outer loop of the program
# EntryConds: 
# - Rascan program is runing (INPROG=0)
# - Rascan state=isStandby

print('\n Re-setting PVs to ModeConfigs... \n',ModeConfigs[0])
load_dict_to_pvs(dict=ModeConfigs[0],eprefix=eprefix,time_scale_factor=1)

# TimeToDo:
# - setup detector Mode parameters
# - syncronise detector position registration for engaged motors (with Encoder Index / or motor)
# 

print('Mode is ready ...')

if doStdoutLog :
    os.system('mv rascanSolveIt.stdout rascanSolveIt.prev.stdout') 


# -------------- Load stack of Addressing Zone Specifiers --------
    
start_Time = time.time()

filename_ = 'ScanConfigs.yaml'
print('\n Reading '+filename_+' ...')
with open(filename_,'r') as stream :
    ScanConfigs = yaml.load(stream)

nOfScanStacks = len(ScanConfigs)
nOfScans = 5   # has to be >0
total_scan_time = 0
# initial values for adavtive accel, effectively no action at first iter
x_motor_ferrMax_accum = 0
y_motor_ferrMax_accum = 0
ferrPixel = 1

i = -1

while i < nOfScanStacks - 1 : 
    #
    # at this point, we ensure that Rascan is atStandby state i.e. dev_State == 2
    # Therefore DEV_SOLVED is set to 0, and PVs are not locked
    #
    #
    i += 1
    
    set_pv_wait(eprefix, ':DEV_CMD', consts.stStandby, ':DEV_STATE', consts.stStandby, 5, '', False)
    
    print('=======> running ScanStack {} of {}'.format(i+1,nOfScanStacks))
    
    #ScanConfigs[i]['D1_IN_HLM'] = str(numpy.maximum(float(ScanConfigs[i]['D1_SPAN']) * 0.9 / 2, float(ScanConfigs[i]['D1_IN_HLM'])))
    #ScanConfigs[i]['D1_OUTHDM'] = str(numpy.maximum(float(ScanConfigs[i]['D1_SPAN']) * 0.1 / 2, float(ScanConfigs[i]['D1_OUTHDM'])))
    
    
    # """ ******************************* Adaptive **************************
    param = 'A1_HLM'
    ferrFact = (x_motor_ferrMax_accum / (nOfScans+1))
    ferrTol = ferrPixel * 4

    if  (ferrFact > ferrTol) :
        dum = ModeConfigs[0][param]*(ferrTol)/ferrFact
        print('ferr was too high= {:f} suggesting to reduce {} to {:f}'.format(ferrFact, param,dum),end="")
        if doAdaptAccel :
            ModeConfigs[0][param] = dum
            caput(eprefix + ':' + param, dum)
            print('...done.')
        else :
            print()

    param = 'A2_HLM'
    ferrFact = (y_motor_ferrMax_accum / (nOfScans+1))
    ferrTol = ferrPixel / 4

    if  (ferrFact > ferrTol) :
        dum = ModeConfigs[0][param]*(ferrTol)/ferrFact
        print('ferr was too high= {:f} suggesting to reduce {} to {:f}'.format(ferrFact, param,dum),end="")
        if doAdaptAccel :
            ModeConfigs[0][param] = dum
            caput(eprefix + ':' + param, dum)
            print('...done.')
        else :
            print()
    # ******************************* Adaptive  ************************** """ 
    
    
    print('\n Setting PVs to ScanConfig... \n',ScanConfigs[i])
    load_dict_to_pvs(dict=ScanConfigs[i], eprefix=eprefix,time_scale_factor=1)
    
    # Now the new ScanConfig is loaded and ready to be solved...
    # Load up Sequencer (SCAN RECORD)
    # but can still be done, e.g. when the stdout file is needed:
    if doStdoutLog :
        print('\n Solving optimized rascan and setting up rascan parameters...')
        start_Time = time.time()
        os.system('rascanSolveIt.py '+eprefix + ' >> rascanSolveIt.stdout')

    # --------------------------------------------------------

    print('\n Validating rascan in PMAC...')
    start_Time = time.time()
    
    #set_pv_wait(eprefix, ':DEV_CMD', consts.stCompiled, ':DEV_STATE', consts.stCompiled, 5, ':DEV_ERROR', False)
    # time.sleep(0.5)
    devErrorFlag=0
    
    # run scan motion (i.e. skip in a dry run)

    if devErrorFlag:
        print('Trajectory validation returned error: '+caget(eprefix+':DEV_ERROR',as_string=True))
    else:
        # Now goReady to verify & lock scan zone
        msg=''
        while msg == '':
            successful_,msg=set_pv_wait(eprefix, ':DEV_CMD', consts.stReady, ':DEV_STATE', consts.stReady, 5, ':DEV_ERROR', False)
            if not successful_:
                print('ERROR ~!')
                print(msg)
                input('press [enter] to retry')

                msg=''
        
        tScan = caget(eprefix+':T_SCAN', as_numpy=True)
        total_scan_time += tScan*nOfScans
        
        if dryRun:
            set_pv_wait(eprefix, ':DEV_CMD', consts.stStandby, ':DEV_STATE', consts.stStandby, 2, ':DEV_ERROR')
        else:
            j = -1
            # loop through the stack here
            while j < nOfScans - 1:
                j += 1
                # reset line No to zero or a preset value
                PVLineN.put(0)
                print('\n Moving to start position...')
                start_Time = time.time()
                set_pv_wait(eprefix, ':DEV_CMD', consts.stSet, ':DEV_STATE', consts.stSet, 50, ':DEV_ERROR', False)

                # At this point, Rascan is standby to run the scan.
                # directly transition Rascan or
                # leave it to a scan_record with preset trasnforms to execute and control the scan
                
                # TEST CODE remove 
                # hardcoded
                caput('SR05ID01FST02:X_MTR:FEMAXRESET.PROC',1)
                caput('SR05ID01FST02:Y_MTR:FEMAXRESET.PROC',1)
                # TEST CODE remove 
                
                print('----------------> running Scan {} of {}'.format(j+1,nOfScans))
                start_Time = time.time()


                # use SCAN record or other sequencer to run rascan as 1D mover, 
                # :RUN is a mover which moves from 0 to 1, as the 2D rascan completes. 
                
                #caput(eprefix+':RUN', 1, wait=False, timeout=2 )
                
                set_pv_wait(eprefix, ':DEV_CMD', consts.stRunning, ':DEV_STATE', consts.stRunning, 2, ':DEV_ERROR')
                
                # scan is running, loop and wait
                
                while PVdevState.value > consts.stReady:
                    time.sleep(0.1)
                    
                    """
                    if PVdevState.value == consts.stRunning :
                        # demonstrate pause action here
                        print('pausing...')
                        PVdevCMD.put(consts.stSet)
                        time.sleep(2)
                    else :
                        if PVdevState.value == consts.stSet :
                            print('unpausing...')
                            PVdevCMD.put(consts.stRunning)
                    """
                # Alternatively, directly use :DEV_CMD -> goRunning and wait for scan to finish...
                # if an error with any of the motors happens, rascan stops with
                # stale states which remains undetected within Rascan
                # set_pv_wait(eprefix, ':DEV_CMD', consts.stRunning, ':DEV_STATE', consts.stCompiled, 5000, ':DEV_ERROR')
                # -------------------------------
                
                # TEST CODE remove 
                #hardcoded: eventually do this via rascan.pmc
                x_motor_ferrMax_accum += caget('SR05ID01FST02:X_MTR:FERRORMAX',as_numpy=True)
                y_motor_ferrMax_accum += caget('SR05ID01FST02:Y_MTR:FERRORMAX',as_numpy=True)
                ferrPixel = float(ScanConfigs[i]['D2_STEP'])
                # TEST CODE remove 
                
                
                
                
    
    # Scan is returned, check if there is an error or not
    #input('press [enter] to continue ...')

print('total scan time={}'.format(total_scan_time))


if doStdoutLog :
    # copy output stdout file to time-stamped file
    now = datetime.datetime.now()
    timeStamp = now.isoformat()
    stdoutFileName = 'rscnSlvt_'+timeStamp+'_.stdout'
    os.system('cp rascanSolveIt.stdout ' + stdoutFileName)

filename_ = 'ScanConfigs.dump.yaml'
print('\n dumping actual scan configs to '+filename_+' ...')
with open(filename_,'w+') as stream :
    stream.write(yaml.safe_dump(ScanConfigs))
    stream.close()

print('\n Terminating rascan...')
start_Time = time.time()
# quit gracefully
set_pv_wait(eprefix, ':DEV_CMD', consts.stTerminated, ':EN_INPROG', 0, 10, '', do_no_status_check=True, do_verbose=False)
# set command to standby to prevent looping back to terminated next time program starts
set_pv_wait(eprefix, ':DEV_CMD', consts.stStandby, ':EN_INPROG', 0, 10, '', do_no_status_check=True, do_verbose=False)

# abort and set the CS to 0 anyways
set_pv_wait(eprefix, ':ABORT.PROC', 1, ':EN_INPROG', 0, 5, '', do_no_status_check=True, do_verbose=False)
