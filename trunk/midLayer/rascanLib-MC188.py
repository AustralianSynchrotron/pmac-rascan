
import time
import sys
import numpy
from epics import caget, caput
from xlrd import open_workbook
import re

#---------------------------
class consts:
    # Inner-skip accel. exceeds limits[]()
    era1In=5    
    # inner-skip range non-consistent[]()
    erd1In=2    
    # reserved[]()
    erERROR4=4  
    # []()
    erERROR8=8  
    # No Fault[]()
    erNONE=0    
    # Motion range exceeds Fly axis limits[]()
    erRange1=6  
    # Motion range exceeds Inc. axis limits[]()
    erRange2=7  
    # []()
    erStandby=9 
    # trajectory builder error[]()
    erTrjBuilder=3  
    # Velocity parameters non-consistent[]()
    erv1Delta=1

    erSolution=21

    # []()
    stBlank=0
    # Trajectory is compiled[]()
    stCompiled=3
    # State: coordinate system setup[]epics(base)
    # stCSSetup x
    # []epics(base)
    # stGlobal  x
    # []epics(base)
    # stInProg  x
    # State: MONITOR[]epics(base)
    # stMonitor x
    # Setup is being verified[]()
    stReady=4
    # rscn program terminated via CTRL[]()
    stReserved=8
    # []()
    stResetting=9
    # Running scan[]()
    stRunning=7
    # Compiled and moved into position to start scan[]()
    stSet=6
    # Moving to begin position to start scan[]()
    stSetting=5
    # Standby, waiting for user setup[]()
    stStandby=2
    # []()
    stTerminated=1
    # []epics(base)
    # stTrajSet x

    # Low limit for tMid[s]()
    tMidLLM=0.05
    # Absolute high limit for time vars[]()
    tAHLM=4.095
    # absolute low limit for time vars[]()
    tALLM=0.001
    # High limit for time vars[s]()
    tHLM=4

    # Hack:
    pE2Res=0.0001 # ERES of the d2 motor... not easily accessible outside of pmac!

#global start_Time, backMsg
# global stCompiled, erNONE, devError

# ---------------

def load_dict_to_pvs(dict,eprefix,time_scale_factor):
    
    caput(eprefix + ':' + 'EN_AUTOSYNC', 0)
    
    for pvName, pvValue in dict.items():
        
        if pvName[1] == 'A' : # acceleration
            _factor = time_scale_factor**2
        if pvName[1] == 'V' : # velocity
            _factor = time_scale_factor
        else :
            _factor = 1        
        
        pvValue = str(  float(pvValue)/_factor )
        
        caput(eprefix + ':' + pvName, pvValue)
    
    time.sleep(0.5)
    caput(eprefix + ':' + 'EN_AUTOSYNC', 1)
# --------------


def load_excel_to_dicts(file_name, header_row, first_col):

    wb = open_workbook(file_name)

    dicts = []

    sheet = wb.sheet_by_name('PVSettings')
    number_of_rows = sheet.nrows
    number_of_columns = sheet.ncols
    rows = []
    for row in range(2, number_of_rows):
        values = []
        keys = []
        scan = ''
        for col in range(2, number_of_columns):
            value = (sheet.cell(row, col).value)
            key = str(sheet.cell(1, col).value)
            try:
                value = str((value))
            except ValueError:
                pass
            finally:
                values.append(value)
                keys.append(key)
                scan += key + '=' + value + ' , '
                # print (item)
        scandict = dict(re.findall(r'(\S+)=(".*?"|\S+)', scan))
        dicts.append(scandict)
    return dicts
# ---------------------------


def set_pv_wait(epics_prefix, set_pv, set_val, get_pv, get_val, timout_sec, fault_pv, do_no_status_check=False, do_verbose=False):
    msg = ''
    if len(set_pv) > 0:
        set_pv = epics_prefix + set_pv
    else:
        set_pv = ''
    if len(get_pv) > 0:
        get_pv = epics_prefix + get_pv
    else:
        get_pv = set_pv + ':RBV'
    if not str(get_val).isnumeric():
        get_val = set_val
    if len(fault_pv) > 0:
        fault_pv = epics_prefix + fault_pv
    _t = 0
    _st = 99

    if do_verbose:
        print('set ' + set_pv[len(epics_prefix):] + " = ", set_val, end="", flush=True)
        print('    waiting for ' + get_pv[len(epics_prefix):] + ' == ', get_val)

    start_Time = time.time()

    caput(set_pv, set_val)
    RBV = caget(get_pv)
    while RBV != get_val:

        _t = (time.time() - start_Time)
        if _t > timout_sec:
            msg = '** Error: TIMEOUT **'
            break

        time.sleep(0.5)

        RBV = caget(get_pv)

        if len(fault_pv) > 0:
            _faultVal = caget(fault_pv)
            if (0 < _faultVal) and (_faultVal < 9):
                msg = '** Error:  FAULT **' + str(_faultVal)
                break

        str1 = "... [{0:.1f}s]".format(_t)
        back = "\b" * len(str1)

        if do_verbose:
            print(str1, end="", flush=True)
            # time.sleep(0.1)
            print(back, end="")
            # print('.',end="",flush=True)

    if not do_no_status_check:
        _st = caget(get_pv + '.STAT', as_string=True)
        if _st != 'NO_ALARM':
            msg = msg + '** STAT=' + _st + ' **'

    msg = '= ' + str(RBV) + ' ' + msg + " T: {0:.3f} \n-----------------------".format(_t)

    return msg
# ---------------------------

def print_stats(stats,scope_,format_string='={:f} ', n_in_row = 100, indent_str = '') :
    
    print(indent_str,end="")
    n = 1
    for stat_ in stats:
        print(stat_ + format_string.format(eval(stat_,scope_)),end="")
        n += 1
        if n > n_in_row :
            print('\n'+indent_str,end="")
            n = 1
    print()
# ---------------------------


def sysMessage(wave_pv,new_msg,flush=False,truncate_str = ' ...') :
    success_ = False
    try:
        max_len = int(caget(wave_pv+'.NELM',as_numpy=True)) - len(truncate_str) - 1
        if max_len > 1 :
            if not flush :
                old_msg = '\n'+ caget(wave_pv,as_string=True)
            else :
                old_msg = ''
            final_msg = new_msg + old_msg
            if len(final_msg) > max_len :
                final_msg = final_msg[:max_len] + truncate_str

            caput(wave_pv,final_msg)
            success_ = True
    except :
        print('\nERROR -> failed adding system message')
    finally :
        return success_
     
# ---------------------------

def rscnSolver(eprefix,verbose_level=1):
   # Double Optimization solver

    # solving rascan dblOpt
    # solution for rascan
    # this routine targets minimising turnaround time: tIn + (tEdge/2)
    # with these constraints:

    # accelerations: ax1 and ax2 trajectoeries maximum acceleration are within the limits: a2HLM

    # ax1 velocity: at v1Scan within tolerance v1TolRatio, while inside the image
    # range specified by d1Span

    # ax2 position: pixel range defined by d2Step within tolerance
    # d2TolRatio while inside the image range specified by d1Span

    # NOTE: turaround overscan d1Out might not be minimized
    #

    solver_version = 'v2.1902'
    
    # % Solver parameters initial point and learning Factor

    iterMax = 99
    
    tSolverMin = 1e-4 * 2
    
    t_erTol = 1e-6
    v1_ErTol = 1e-5
    a1_ErTol = 1e-5
    d2_ErTol = 20e-9

    dnF_ = '{0:2.4e}'
    
    indent_constraint = '   | '
    indent_stats = '   = '
    indent_change = '   > '
    

    #{(0,'silent'),(1,'progress'),(2,'step changes'),(3,'steps and constraints'),(4,'detail'),(5,'debug')}
    #verbose_level = 5

    if verbose_level > 0 : print('Rascan solver '+solver_version)
    
    # % interface definition
    a1HLM = caget(eprefix+':A1_HLM',as_numpy=True)
    a2HLM = caget(eprefix+':A2_HLM',as_numpy=True)
    d1InHLM = caget(eprefix+':D1_IN_HLM',as_numpy=True)
    d1Span = caget(eprefix+':D1_SPAN',as_numpy=True)
    d2Step = caget(eprefix+':D2_STEP',as_numpy=True)
    d2TolRatio = caget(eprefix+':D2_TOLRATIO',as_numpy=True)
    devCMD = caget(eprefix+':DEV_CMD',as_numpy=True)
    devError = caget(eprefix+':DEV_ERROR',as_numpy=True)
    devState = caget(eprefix+':DEV_STATE',as_numpy=True)
    tInLLM = caget(eprefix+':T_IN_LLM',as_numpy=True)
    j1HLM = caget(eprefix + ':J1_HLM', as_numpy=True)
    j2HLM = caget(eprefix + ':J2_HLM', as_numpy=True)
    v1Scan = caget(eprefix+':V1_SCAN',as_numpy=True)
    #v1TolRatio = caget(eprefix+':V1_TOLRATIO',as_numpy=True)
     
    v1ScanHLM = caget(eprefix+':V1_SCANHLM',as_numpy=True)
    v1ScanLLM = caget(eprefix+':V1_SCANLLM',as_numpy=True)
    d1OutHDM = caget(eprefix+':D1_OUTHDM',as_numpy=True)

    if verbose_level > 1 : 
        print('Inputs:')
        print('d1Span={3:f}, v1ScanHLM={11:f}, v1ScanLLM={12:f}, d1InHLM={2:f}, d1OutHDM={13:f}, d2Step={4:f}, d2TolRatio={5:f}\n'
          'a1HLM={0:f}, a2HLM={1:f}, tInLLM={9:f} \n devState={8:0} devCMD={6:0}, devError={7:0}'
          .format(a1HLM,a2HLM,d1InHLM,d1Span,d2Step,d2TolRatio,devCMD,devError,devState,tInLLM,v1Scan,v1ScanHLM,v1ScanLLM,d1OutHDM))

    #%% pre validate the case
    validation_conditions = ['v1ScanLLM*tInLLM > d1InHLM', 'd2TolRatio*d2Step*6 < a2HLM*tInLLM**2'];
    for validation_condition_ in validation_conditions :
        if eval(validation_condition_) :
            devError = consts.erSolution
            caput(eprefix +':DEV_ERROR', devError)
            msg_ = 'ERROR -> inconsistent requirements: '+validation_condition_
            if verbose_level > 0 : print(msg_)
            sysMessage(eprefix+':SYS_WARNS',msg_)
            sysMessage(eprefix+':SYS_ERRORS',msg_) 
            return devError


    # % ---------------- BEGIN BLOCK INIT

    
    a1Max = a1HLM
    a1MaxOld = a1Max
    
    if (d2Step > 0):
        a2Max = abs(a2HLM)
    else:
        a2Max = -abs(a2HLM)

    a2MaxOld=a2Max

    a1Out = a1Max

    v1Scan = v1ScanHLM
    v1Out = v1Scan
    v1Diff = 0
    v1Diff_d1In_min = 0

    # establish d1In at its max
    d1In_max = d1Span/2*0.95
    if (d1In_max > d1InHLM):
        d1In_max = d1InHLM

    tIn_max = tInLLM

    # set tIn to satisfy d1In: conservative setting diregarding the v1Diff variations...
    dum = d1In_max/(v1ScanLLM)
    if dum > tIn_max :
       tIn_max = dum

    # Conservative assumption: regardless of VTol
    # Now set d2In at its max possible with tIn and a2Max... !!!

    d2In_max = (a2Max*tIn_max*tIn_max)/6
    dum=d2Step*d2TolRatio
    if d2In_max >  dum : # start at highest
        d2In_max = dum
        tIn_max = numpy.sqrt(6)*numpy.sqrt(d2In_max/a2Max)

    tIn = tIn_max
    tInOld = tIn

    d1In = d1In_max

    d2In = d2In_max

    # d2In is good only if its not making ax2 too fast, which will be indicated by a (tEdge/2) < 0

    tOut = 0
    tOutOld = tOut

    tEdge = 0
    tEdgeOld = tEdge

    tOvrHd = 10000
    tOvrHdOld = tOvrHd

    i = 1
    er = 100
    # ---------------- END BLOCK INIT
    # numerically solving double optimization problem
    erLog = numpy.zeros((iterMax, 6))

    thisIterChanges = ''

    if verbose_level > 0 : print('Iterations ',end="")
    if verbose_level > 2 : print('\n{')

    # % ---------------- BEGIN BLOCK ITER
    while (i < iterMax) and ((er > t_erTol) or (thisIterChanges)) :

        if verbose_level > 2 : print('{:3d}'.format(i),end="")
        if verbose_level > 0 : print('. ',end="")
        if verbose_level > 3 : print()

        pdelta = d2Step / a2Max

        # Find tIn based on ax2 requirement
        tIn = numpy.sqrt(6) * numpy.sqrt(d2In / a2Max) # min required for ax2

        #% Jerk calculations
       
        j1HLM_opt = a1Out/tIn
        
        v1Diff_j_max=j1HLM*tIn*tIn/2
        #%v1Diff_j_min = (tIn*numpy.sqrt(-8*j1HLM*v1Scan+j1HLM**2*tIn**2+4*a1Out**2)-j1HLM*tIn**2)/4
        
        #% v1Diff limit calculations

        v1Diff_max=a1Out*tIn/2
        _v1Diff_msg = 'j1In'
        # v1Diff wich makes j1Out minimal
        dum = v1Scan - v1ScanLLM
        if v1Diff_max > dum :
            v1Diff_max = dum
        
        if v1Diff_max > v1Diff_j_max :
            v1Diff_max = v1Diff_j_max
            _v1Diff_msg = 'j1HLM'

        v1Diff_d1In_min = 3*v1Scan-(3*d1In_max)/tIn
      
#        if v1Diff_d1In_min > v1Diff_max :
#            dum = dum
#            if verbose_level > 5 : print('v1Diff_d1In_min {:f} > v1Diff_max {:f}'.format(v1Diff_d1In_min,v1Diff_max))
        
        ## find v1Scan limit
        v1Scan_max = v1ScanHLM
        _v1Scan_msg = ''
        dum = v1Diff/3 + d1In_max/tIn
        if dum < v1ScanLLM : dum = v1ScanLLM
        if v1Scan_max < dum :
            v1Scan_max = dum
            _v1Scan_msg = 'd1In_max'

        ## limit v1Scan_max for d1OutHDM
        dum = v1Diff+(numpy.sqrt(3)*(2*v1Diff+a1Out*tIn)*numpy.sqrt(d1OutHDM/(tIn*(v1Diff+a1Out*tIn))))/2;
        if dum < v1ScanLLM : dum = v1ScanLLM
        if v1Scan_max > dum :
            v1Scan_max = dum
            _v1Scan_msg = 'd1OutHDM'

        #% v1Out optimal value to minimize tLine
        v1Out_opt = (numpy.sqrt(2)*numpy.sqrt(2*v1Diff+a1Out*tIn)*numpy.sqrt(tIn*v1Diff+2*d1Span)-4*numpy.sqrt(tIn)*v1Diff)/(4*numpy.sqrt(tIn))
        v1Diff_op_tLine = (2**(3/2)*numpy.sqrt(v1Scan)*numpy.sqrt(2*v1Scan+a1Out*tIn)-a1Out*tIn)/2
        
        #tLine monotonically increases with v1Diff if tEdge > 0
        if tEdge > 0 : 
            v1Diff_op_tLine = v1ScanHLM
        
        if v1Diff_op_tLine < 0 :
            v1Diff_op_tLine = 0

        if v1Diff_op_tLine > v1Scan - v1ScanLLM :
            v1Diff_op_tLine = v1Diff_op_tLine #v1Scan - v1ScanLLM

        if v1Diff_op_tLine < v1Diff_max :
            v1Diff_max = v1Diff_op_tLine
            _v1Diff_msg = 'tLine'

        if verbose_level > 3 :
            if _v1Diff_msg : 
                print(indent_constraint+'v1Diff_max={:f}'.format(v1Diff_max)+ ' by ' + _v1Diff_msg)
            if _v1Scan_msg : 
                print(indent_constraint+'v1Scan_max={:f}'.format(v1Scan_max)+' by ' + _v1Scan_msg)
 
        #% limit v1Diff
        if v1Diff > v1Diff_max :
            # Exceeding VTol, then use up VTol and reduce Inner Skip
            v1Diff = v1Diff_max
            if verbose_level > 3 : print(indent_constraint+'v1Diff:=v1Diff_max {:f}'.format(v1Diff_max))

        if v1Scan > v1Scan_max :
            # Exceeding VTol, then use up VTol and reduce Inner Skip
            v1Scan = v1Scan_max
            if verbose_level > 3 : print(indent_constraint+'v1Scan:=v1Scan_max {:f}'.format(v1Scan_max))
        
        
        #% dependent variables calculations

        # (rscn_WellCond)
        # based on VAdd, tIUSkip behaviour changes: between the limits, tIn
        # has an omptimal value which optimizes the tOvrHd. with VAdd higher
        # than Hi, tIn willincrease tOvrHd, so it ashall not be
        # increased. with VAdd lower than min, tIn can be increased.
        # Therefore, this tIn gives a high limit for

        v1Out = v1Scan - v1Diff

        alpha = (a1Out * tIn + 2 * v1Diff)
        tOut = (2 * v1Out * tIn) / alpha
        d1Out=(4*(alpha-v1Diff)*v1Out*v1Out*tIn)/(3*alpha*alpha)

        # Find (tEdge/2) for ax2

        ptau = (numpy.sqrt(tIn ** 2 + 24 * pdelta) - 3 * tIn) / 8                        #(rscn_ptau_ax2)
        # ptau is monotonically decreasing to tIn right?
        tEdge = 4 * ptau - 2 * tOut

        # to check how the total time is converging towards its minimum

        tOvrHd = 2*(tOut + (tEdge / 2) + (v1Diff * tIn) / (3 * v1Scan))
        tLine = d1Span/v1Scan+tOvrHd
        #%pdeltatOv = tOvrHd - tOvrHdOld
        
        d1In = (tIn*(3*v1Scan-v1Diff))/3;
        
        #myTextFormat = '\n      tLine-tEdge=%3.5f tEdge=%3.5f v1Out %3.4f|%3.4f| v1Diff %3.4f < %3.4f |%3.4f| <%3.4f d1In=%3.4f\n      '
        #fprintf(myTextFormat,tLine-tEdge,tEdge,v1Out,v1Out_opt,v1Diff_d1In_min,v1Diff,v1Diff_op_tLine,v1Diff_max,d1In)    
        thisIterChanges = ''

        # dtLine/dv1Scan v1Diff=cte
        dtLine_dv1Scan = (8*tIn*v1Scan**2-2*tIn*v1Diff**2+(-a1Out*tIn**2-4*d1Span)*v1Diff-2*a1Out*d1Span*tIn)/((4*v1Diff+2*a1Out*tIn)*v1Scan**2)
        #fprintf('dtLine_dv1Scan=%4.4f ',dtLine_dv1Scan)

        #% dtLine/dv1Diff v1Scan=cte

        dtLine_dv1Diff= -(16*tIn*v1Scan**2+8*a1Out*tIn**2*v1Scan-4*tIn*v1Diff**2-4*a1Out*tIn**2*v1Diff-a1Out**2*tIn**3)/((8*v1Diff**2+8*a1Out*tIn*v1Diff+2*a1Out**2*tIn**2)*v1Scan)
        #fprintf('dtLine_dv1Diff=%4.4f ',dtLine_dv1Diff)
        
        dtLine_dv1Diff_Edge = tIn/(2*v1Scan)
        #(f_tIn_op_tLine_Edge)
        if v1Diff > 0 :
            tIn_opt_tLine = (2**(3/2)*numpy.sqrt(3)*numpy.sqrt(pdelta)*(v1Scan-v1Diff))/(numpy.sqrt(v1Diff)*numpy.sqrt(2*v1Scan-v1Diff))
        else :
            tIn_opt_tLine = tIn_max
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^       
        if verbose_level > 3 :
            stats = ['tLine','tEdge','v1Scan','v1Out']
            print_stats(stats,locals(),indent_str=indent_stats)

 
        if verbose_level > 4 :
            stats = ['tIn_opt_tLine','v1Diff_d1In_min','d1Span-d1In*2','d1InHLM-d1In','tIn-tInLLM','d1OutHDM-d1Out','d2In/d2Step','dtLine_dv1Scan','dtLine_dv1Diff','dtLine_dv1Diff_Edge']
            print_stats(stats,locals(),indent_str=indent_stats)

        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        #%% case ax1 turn-round is quicker than ax2
        if (tEdge > tSolverMin) :
            
        #% ax2 is SLOWER than ax1... try optimising d1Out
            #%? ax2 is too slow, 
            #% :decrease currently positive tEdge to minimum
            
            
            if (not thisIterChanges) and (v1Diff_d1In_min > v1Diff) :
                #%% this means that d1In is too large, and v1Diff can probably be increased to help that

                dumFactor = 0.1
                dum = v1Diff_d1In_min
                if (dum > v1Diff_max) : dum = v1Diff_max

                dum = (1-dumFactor)*v1Diff + (dumFactor)*dum

                if abs(v1Diff - dum) > v1_ErTol :
                    v1Diff=dum
                    thisIterChanges += 'v1Diff:={:f} for d1In'.format(v1Diff)
                else :
                    #no decision no change!
                    v1Diff=v1Diff
            
            if (not thisIterChanges) and (d1In > d1In_max) :
                #%% Directly reduce tIn, by reducing d2In, to decrease d1In            
                dumFactor = 0.8
                
                d2In_max = (3*a2Max*d1In_max**2)/(2*(3*v1Scan-v1Diff)**2)
                dum = (1-dumFactor)*d2In + (dumFactor)*d2In_max
                
                #%dum = d2In*dumFactor
                
                if abs(d2In - dum) > d2_ErTol :
                    d2In = dum
                    thisIterChanges += 'd2In:={:f} for d1In'.format(d2In)
                    #% if VTol is used to make ax1 return faster, then
            if (not thisIterChanges) and (v1Diff > 0) :
                #%% Use LESS inner decceleration on Fly axis to decrease d1Out
                #% slowing down ax1, 
                dumFactor = 2/3
                pdeltaT=(tEdge/2)*dumFactor
                tOutdum=tOut+pdeltaT #% eat up part of (tEdge/2) into tOut

                dum=-(tIn*(a1Out*tOutdum-2*v1Scan))/(2*(tOutdum+tIn))

                if (dum > v1Diff_d1In_min) :
                    if (dum < 0) :
                       v1Diff = 0
                       thisIterChanges += 'v1Diff:={:f} for tEdge'.format(v1Diff)
                    else :
                        if (dum < v1Diff_max) :
                            v1Diff=dum
                            thisIterChanges += 'v1Diff:={:f} for tEdge'.format(v1Diff)
                        else :
                            #no decision no change!
                            v1Diff=v1Diff

            if (not thisIterChanges) and (d1Out < d1OutHDM) :
                #%% d1Out is good, so try to get rid of the edge by reducing a1Out
                #% this move usually helps j1Out but no gaurantee that this is
                #% under j1HLM. However, at tEdge=0, if j1 is still above the
                #% limit, it may mean that a1 and a2 shall both be scaled back
                dumFactor = 1 - t_erTol
                pdeltaT=(tEdge/2)*dumFactor
                tOutdum=tOut+pdeltaT #eat up part of (tEdge/2) into tOut
                
                dum = (2*tIn*v1Scan+(-2*tOutdum-2*tIn)*v1Diff)/(tIn*tOutdum)
                
                
                _sqrTerm_ = tIn**2*v1Out**2-3*d1OutHDM*tIn*v1Diff
                if _sqrTerm_ < 0 : _sqrTerm_ = 0
                a1Out_min = (2*v1Out*numpy.sqrt(_sqrTerm_)+2*tIn*v1Out**2-6*d1OutHDM*v1Diff)/(3*d1OutHDM*tIn)
                if a1Out_min < (2*v1Diff)/tIn :
                    a1Out_min = (2*v1Diff)/tIn
                
                if dum < a1Out_min :
                    dum = a1Out_min
                    if verbose_level > 3 : print(indent_constraint+'a1Out_min {:f} by d1OutHDM'.format(a1Out_min))

                if abs(a1Out - dum) > a1_ErTol:
                    a1Out = dum
                    thisIterChanges += 'a1Out:={:f} for j1'.format(a1Out)

        #%% case ax2 turn-around is quicker
        #% ax1 is not quick, need to increase currently negative tEdge to 0
        if (tEdge < -tSolverMin) :
            #%% Consider chaniging velocities... based on sensitivity of tLine
            #% don't try to optimise v1Diff for time!!!!  
            #% see version #5 and #4

            #% find max negative sensitivity
            dtLine_max = 0
            if -dtLine_dv1Diff > dtLine_max :
                dtLine_max = -dtLine_dv1Diff
            if (dtLine_dv1Scan > dtLine_max) :
                dtLine_max = dtLine_dv1Scan
            if (dtLine_max == dtLine_dv1Scan) and (v1Out > v1ScanLLM) :
                if verbose_level > 4 : print(indent_constraint+'(dtLine_max == dtLine_dv1Scan) and (v1Out > v1ScanLLM)')

            if (not thisIterChanges) and (v1Out > v1ScanLLM) :
                #%% try increasing v1Diff for least j1Out
                #% this may actually m,ake tEtge more negative but its best time
                #% to do this!
                
                dumFactor = 1 - t_erTol
                
                pdeltaT=(tEdge/2)*dumFactor
                tOutdum=tOut+pdeltaT #% eat up part of (tEdge/2) into tOut
                dum=-(tIn*(a1Out*tOutdum-2*v1Scan))/(2*(tOutdum+tIn))


                if dum < v1Diff_d1In_min :
                    dum = v1Diff_d1In_min

                if dum > v1Diff_max :
                    dum = v1Diff_max
                
                if abs(v1Diff - dum) > v1_ErTol :
                    v1Diff=dum
                    thisIterChanges += 'v1Diff:={:f} for j1'.format(v1Diff)
            
            if (not thisIterChanges) and (dtLine_dv1Scan > 0 ) and (v1Scan > v1Diff + v1ScanLLM) :
                #%% Reduce v1Scan to gain on tLine
                
                dumFactor = 1-v1_ErTol

                dum = v1Diff + v1Out_opt

                dum = (1-dumFactor)*v1Scan + (dumFactor)*dum
                
                if (dum > v1Scan_max) :
                    dum = v1Scan_max
                    if verbose_level > 3 : print(indent_constraint+'v1Scan_max{:f} applied by d1InHLM'.format(v1Scan_max))

                if (dum < v1Diff + v1ScanLLM) :
                    dum = v1Diff + v1ScanLLM

                if abs(v1Scan - dum) > v1_ErTol :
                    v1Scan = dum
                    thisIterChanges += 'v1Scan:={:f} for tLine'.format(v1Scan)
            
            if (not thisIterChanges) and (d1In > d1In_max) :
                #%% Directly reduce tIn, by reducing d2In, to decrease d1In            
                dumFactor = 1-d2_ErTol
                
                d2In_max = (3*a2Max*d1In_max**2)/(2*(3*v1Scan-v1Diff)**2)
                dum = (1-dumFactor)*d2In + (dumFactor)*d2In_max
                
                #%dum = d2In*dumFactor
                
                if abs(d2In - dum) > d2_ErTol :
                    d2In = dum
                    thisIterChanges += 'd2In:={:f} for d1In'.format(d2In)
            
            #%% Reduce a2Max to get rid of tEdge
            if (not thisIterChanges) :
                
                #% ax1 can't be made quicker
                #% Decrease a2Max to increase currently negative tEdge to 0
                dumFactor = 1/8
                dum=abs((6*d2Step)/((tOut+tIn)*(4*tOut+2*tIn+3*dumFactor*tEdge)))
                if dum < a2Max :
                    char_ = ''
                else :
                    dum = a2Max * 0.9
                    char_ = '0.9'

                #% Inorder for the change to be retained, d2In has to change
                #% accordingly. Otherwise tIn will be calculated based on
                #% reduced a2Max and nullify its effect
                d2In = d2In *dum /a2Max
                a2Max = dum            
                
                thisIterChanges += 'a2Max:'+char_+'={:f} for tEdge'.format(a2Max)

        #% report & assert convergence
        if verbose_level > 2 : print(indent_change+thisIterChanges)

        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        erLog[i, 1] = tOut - tOutOld
        tOutOld = tOut#

        erLog[i, 2] = tIn - tInOld
        tInOld = tIn#

        erLog[i, 3] = (tEdge / 2) - (tEdgeOld / 2)
        tEdgeOld = tEdge#

        erLog[i, 4] = a2Max - a2MaxOld
        a2MaxOld = a2Max#

        erLog[i, 5] = tOvrHd - tOvrHdOld
        tOvrHdOld = tOvrHd#


        er = max(abs(erLog[i,:]))
        i = i + 1

    # ---------------- END BLOCK ITER
    # display(v1Out)
    # %

    if verbose_level > 2 : print('}',end="")
    if verbose_level > 0 : print()    
    
    if i == iterMax:
        msg_ = 'ERROR -> solver did not converge in'+' {0:0} iters; deviation={0:.2e}'.format(i, er)
        sysMessage(eprefix+':SYS_ERRORS',msg_)
        if verbose_level > 0 : 
            print(msg_)
        #if verbose_level > 3 : 
            #print(erLog[i - numpy.min(i - 1, 3):i - 1, :])

        lineSubEnd = 0
        devError = consts.erSolution
        d1Out = 0
        m_lineStartTime = 0
        tOvrHd = 0

    if i < iterMax:
        if not (abs(v1Scan - v1Out - v1Diff) < v1_ErTol**2):
            devError = consts.erSolution
            caput(eprefix +':DEV_ERROR', devError)
            msg_ = 'ERROR -> incosistent solution: erSolution'.format(devError)
            if verbose_level > 0 : print(msg_)

            #sysMessage(eprefix+':SYS_WARNS',msg_)
            sysMessage(eprefix+':SYS_ERRORS',msg_)

            return devError

        if verbose_level > 0 : print('Converged within {1:.4e} units after {0:0} iterations'.format(i-1, er))
        # % setup scan shaping via indentation


        # d1IndRFun = numpy.mcat([0, -0.1, 0.1, -0.2, -0.1, -0.2, -0.3])
        # d1IndLFun = numpy.mcat([0, 0.1, -0.1, 0.2, -0.1, -0.2, -0.3])

        #     d1IndLFun=(cos([0:lineEnd/2-2]*pi/(lineEnd/4))-1)*d1Span#
        #     d1IndRFun=d1IndLFun#
        #
        #     d1IndLFun=(-cos([0:lineEnd/2-2]*pi/(lineEnd/4))+1)*d1Span#
        #     d1IndRFun=-d1IndLFun#

        # % run motion program
        lineN = 0


        # Set these vars:
        if verbose_level > 0 : print('Found a solution.')
    #    if verbose_level > 1 : print('T_EDGE={0:.5f}, T_IN={1:.5f}, T_OUT={2:.5f}, V1_DIFF={3:.5f}, V1_SCAN={4:.5f}, A1_OUT={5:.5f},A2_MAX={6:.5f}'.format(tEdge,tIn,tOut,v1Diff,v1Scan,a1Out,a2Max))
        if verbose_level > 0 : print('\nCompiling and validating ...')
        validation_msgs = '';
        
        ## {{------------------- Compiling: PVT calculations

        #global stCompiled, erNONE

        devState = consts.stCompiled
        devError = consts.erNONE

        lineDir = 1 - 2 * (lineN % 2)  #
        v1Out = v1Scan - v1Diff  #
        palpha = (a1Out * tIn + 2 * v1Diff)  #
        d1In = ((3 * v1Out + 2 * v1Diff) * tIn) / 3  #
        d1Out = (4 * (palpha - v1Diff) * v1Out * v1Out * tIn) / (3 * palpha * palpha)  #
        # (o585)
        a2Out = (a2Max * tEdge) / (2 * tOut + tEdge)  #
        # (d2InEq)
        d2In = (a2Max * tIn * tIn) / 6  #
        # (v2InEq)
        v2In = (a2Max * tIn) / 2  #
        d2Out = (tOut * (a2Out * tOut + 2 * a2Max * tOut + 3 * a2Max * tIn)) / 6  #
        v2Out = v2In + tOut * (a2Max + a2Out) / 2  #
        # calculate    Edge    move \
            #    if ESkip is almost zero...keep it there
        # while searching for a min


        d2Edge = (d2Step - 2 * d2In - 2 * d2Out)  #
        if (d2Edge < consts.pE2Res):
            tEdge = 0  #

        if (tEdge < 0.002):
            d2Edge = 0  #
            tEdge = 0  #
            d2Out = d2Step / 2 - d2In  #

        d1IndHLM = (-(consts.tMidLLM + 0.001) * v1Scan + d1Span - 2 * d1In)  #
        d1IndL = 0  #
        d1IndR = 0  #
        tOvrHd = 2*(tOut + (tEdge / 2) + (v1Diff * tIn) / (3 * v1Scan))  #
        ## }} -------------------- Compiling: PVT    calculations \     \


        ## {{-------------------- Compiling: Validation

        if (tEdge < 0):
            devError = consts.erTrjBuilder  #

        if (d2In < 0):
            devError = consts.erTrjBuilder  #

        if (tIn < tInLLM):
            devError = consts.erERROR8

        if (d2Edge < 0):
            devError = consts.erTrjBuilder  #        'd2Edge < 0'  #

        a1In = v1Diff/tIn*2
        if (a1In > a1HLM*1.01):
            devError=consts.era1In


        
        # ----------------------------------------------------------

        # ----------------
        tMid = (d1Span - 2 * d1In) / v1Scan  #
        lineSubEnd = 0  #
        if (tMid > consts.tHLM):
            while (tMid > consts.tHLM):
                lineSubEnd = lineSubEnd + 1  #
                tMid = tMid - consts.tHLM  #
        # now if tMid is too small, add one tHLM to tMid and deduct one from subsegments
            if (tMid < consts.tAHLM - consts.tHLM):
                tMid = tMid + consts.tHLM  #
                lineSubEnd = lineSubEnd - 1  #

        # tMid should be less than PVT acceptable limit
        if (tMid > consts.tAHLM):
            devError = consts.erTrjBuilder  #

        if (tMid < 0.002):
            devError = consts.erd1In  #

        # ------------------------
        if (tIn > consts.tAHLM):
            devError = consts.erTrjBuilder  #

        if (tOut > consts.tAHLM):
            devError = consts.erTrjBuilder  #

        if (tEdge > consts.tAHLM):
            devError = consts.erTrjBuilder  #

        if (d1Span < 2 * d1In):
            devError = consts.erd1In  #


        # }} -------------------- Compiling: Validation

        tLine = 2 * tOut + 2 * tIn + tEdge + tMid + lineSubEnd * consts.tHLM

        er_definitions = {
            '(tEdge < 0)': consts.erTrjBuilder,
            '(d2In < 0)' : consts.erTrjBuilder,
            '(tIn < tInLLM)' : consts.erERROR8,
            '(d2Edge < 0)' : consts.erTrjBuilder,
            '(tMid > consts.tAHLM)' : consts.erTrjBuilder,
            '(tMid < 0.002)': consts.erd1In,
            '(tIn > consts.tAHLM)': consts.erTrjBuilder,
            '(tOut > consts.tAHLM)': consts.erTrjBuilder,
            '(tEdge > consts.tAHLM)': consts.erTrjBuilder,
            '(d1Span < 2 * d1In)': consts.erd1In,
            '(a1In < 0)': consts.erTrjBuilder

        }

        for key, value in er_definitions.items():
            if eval(key):
                validation_msg_ = 'ERROR -> '+key
                validation_msgs = validation_msg_ + ';' + validation_msgs
                if verbose_level > 0 : print('\n', validation_msg_)
                sysMessage(eprefix+':SYS_ERRORS',validation_msg_)
                devError= value


        
        # Verify if jerks are within limits =========================
        j1In = (a1In-0)/tIn
        j1Out = (a1Out-a1In)/tOut

        j2In = ((v2In/tIn*2)-0)/tIn
        j2Out = (a2Out-j2In*tIn)/tOut
        j2Edge = (0-a2Out)/(tEdge/2+consts.tALLM)


        for _axis in ['1','2'] :
            for _dir in ['In','Out'] :
                _varName = 'j' + _axis + _dir
                _limName = 'j' + _axis + 'HLM'
                _varVal = eval(_varName)
                _limVal = eval(_limName)
                if(abs(_varVal)>_limVal):
                    validation_msg_ = 'WARNING -> ' + _varName + '({0:.3e})>({1:.3e})'.format(_varVal, _limVal)
                    validation_msgs = validation_msg_ + ';' + validation_msgs
                    if verbose_level > 0 : print(validation_msg_,end="\n")
                    sysMessage(eprefix+':SYS_WARNS',validation_msg_)

        if (devError == consts.erNONE) :
            caput(eprefix+':T_EDGE',tEdge)
            caput(eprefix+':T_IN',tIn)
            caput(eprefix+':T_OUT',tOut)
            caput(eprefix+':V1_DIFF',v1Diff)
            caput(eprefix +':V1_SCAN', v1Scan)            
            caput(eprefix +':A1_OUT', a1Out)
            caput(eprefix+':A2_MAX',a2Max)
            
            caput(eprefix+':T_LINE',tLine)
            caput(eprefix+':T_OVRHD',tOvrHd)
            
            if verbose_level > 0 :
                print('Trajectory validated.\n')
                if verbose_level > 1 :

                    print('Outputs:')
                    stats = ['tEdge','tIn','tOut','v1Diff','v1Scan','a1Out','a2Max']
                    print_stats(stats,locals(),format_string='={0:4.4e} ')
                    print()

                if verbose_level > 2 :
                    print('Trajectory details:')
                    stats = ['d1In','d1Out','v1Out','d2In','d2Out']
                    print_stats(stats,locals(),format_string='={0:4.4e} ')
                    print()


                print('Overall time per line (T_LINE)={0:.4f}'.format(tLine))
                print('Actual overhead time per line (T_OVRHD)={0:.4f}'.format(tOvrHd))
                print('Overhead to total: {0:2.0f}%'.format(100*tOvrHd/tLine))
        else :
            if verbose_level > 0 :
                print('Trajectory not validated!')
        #if validation_msgs :
            #sysMessage(eprefix+':SYS_WARNS','Validation: '+validation_msgs)

    caput(eprefix +':DEV_ERROR', devError)
    return devError

