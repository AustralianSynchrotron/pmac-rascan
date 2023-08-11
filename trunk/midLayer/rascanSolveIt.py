#!/usr/bin/env python
#
# $File: //ASP/tec/mc/pmacRascan/trunk/midLayer/rascanSolveIt.py $
# $Revision: #7 $
# $DateTime: 2019/03/07 11:29:20 $
# Last checked in by: $Author: afsharn $
#
# Description
# <description text>
#
# Copyright (c) 2019 Australian Synchrotron
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# Licence as published by the Free Software Foundation; either
# version 2.1 of the Licence, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public Licence for more details.
#
# You should have received a copy of the GNU Lesser General Public
# Licence along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
#
# Contact details:
# nadera@ansto.gov.au
# 800 Blackburn Road, Clayton, Victoria 3168, Australia.
#

helpText = '''
    Solves scan kinematics for rascan 2.0 model 
    
    Syntax:
        rascanSolveIt.py <epicsPrefix> [-v0..5]
    
    -v verbose level 0..5
    Inputs directly obtained from epics PV's:
        eprefix+':A1_HLM', ':A2_HLM', ':D1_IN_HLM', ':D1_SPAN', ':D2_STEP', ':D2_TOLRATIO', ':DEV_CMD'
        , ':DEV_ERROR', ':DEV_STATE', ':T_IN_LLM', ':J1_HLM', ':J2_HLM', ':V1_SCAN', ':V1_TOLRATIO'
        
    Outputs are caput to PV's:
    
        ... ':A2_MAX', ':T_EDGE', ':T_IN', ':T_OUT', ':V1_DIFF', ':A1_OUT'
    

    Please email your queries to nadera@ansto.gov.au    

  '''


import sys
import time
import datetime
from epics import caget, caput

import rascanLib as rscn




def main():
    '''Main entry point of the script.'''
    errCode = 99

    if len(sys.argv) < 2:
        sys.argv.append('WORKSHOP01:RSCN')
        print('default test prefix is used')
    
    verbose_level = 5
    if len(sys.argv) > 2:
        dum = sys.argv[2]
        if dum[0:2] =='-v' :
            verbose_level = int(float(dum[2]))

    if verbose_level>0 : 
        print('\n{{\n===============================================')
        print(datetime.datetime.now().isoformat())
    
    if len(sys.argv) > 1:
        eprefix = sys.argv[1]
        rscn.sysMessage(eprefix+':SYS_WARNS','rascanSolveit is running...',flush=True)
        rscn.sysMessage(eprefix+':SYS_ERRORS','',flush=True)
        devSlvd = caget(eprefix+':DEV_SLVD:RBV')
        inProg = caget(eprefix+':EN_INPROG',as_numpy=True)
        devState = caget(eprefix+':DEV_STATE',as_string=True)

        msg_ = ''
        if (inProg < 1) : 
            msg_ = 'program not running'
        else :
            if (devState != 'isStandby') : 
                msg_ = 'not isStandby'
            else:
                if (devSlvd == 1) : msg_ = 'solution not reset'
         
        if msg_ :
            msg_ = '! ' + msg_
            # A solution is loaded, ignore and quit with errCode

            if verbose_level>0 : print('\nWARNING -> '+msg_)
            rscn.sysMessage(eprefix+':SYS_WARNS', msg_)
            errCode = 98
        else :
            # A solution is needed

            caput(eprefix+':EN_AUTOSYNC', 1)
            
            # --------------------------------------------------------
            msg_ = 'Compiling ' +eprefix
            rscn.sysMessage(eprefix+':SYS_WARNS',msg_,flush=True)
            if verbose_level>0 : 
                print(msg_)
                print('---------------------')
            start_Time = time.time()

            errCode = rscn.rscnSolver(eprefix,verbose_level)
            
            if verbose_level>0 : 
                print('----------------------')

            if errCode == 0 :
                # --------------------------------------------------------
                # set and wait for the mapped parameter to be synched (...SCD=1)
                if verbose_level > 3 : print('\nSOLVED flag was set ('+eprefix+':DEV_SLVD'+')')
                rscn.sysMessage(eprefix+':SYS_WARNS','Compiled.',flush=False)
                caput(eprefix+':DEV_SLVD', 1)
            else :
                msg_ = 'ERROR -> solver error='+str(errCode)
                rscn.sysMessage(eprefix+':SYS_ERRORS',msg_)
                rscn.sysMessage(eprefix+':SYS_WARNS','Returned with error')
                if verbose_level > 0 : print('\n '+msg_)
    else:
        print(helpText)
    
    if verbose_level>0 : 
        print(datetime.datetime.now().isoformat())
        print('===============================================\n}}\n')
    
    return errCode

if __name__ == '__main__':

    sys.exit(main())

