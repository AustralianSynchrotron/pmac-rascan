   # % Solver parameters initial point and learning Factor



    iterMax = 100
    erTol = 1e-6
    tSolverMin = 1e-4 * 2


    # % ---------------- BEGIN BLOCK INIT

    
    a1Max = a1HLM
    a1MaxOld = a1Max
    
    if (d2Step > 0):
        a2Max = abs(a2HLM)
    else:
        a2Max = -abs(a2HLM)
    a2MaxOld = a2Max

    a1Out = a1Max
    v1Out = v1Scan
    v1Diff = 0

    d1In = d1Span / 2 * 0.95
    tIn = tInLLM

    # Check if d1 is within the limits
    if (d1In > d1InHLM):    # && (v1TolRatio>0)
        d1In = d1InHLM
    dum = d1In / v1Scan
    if dum > tIn:
        tIn = dum
    tInOld = tIn

    # Conservative assumption: regardless of VTol

    d2In = (a2Max * tIn * tIn) / 6
    dum = d2Step * d2TolRatio
    if d2In > dum:            # start at highest
        d2In = dum
    d2InOld = d2In
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

    firstPass = 1
    sys.stdout.write('solving')
    # % ---------------- BEGIN BLOCK ITER
    while (i < iterMax) and (er > erTol):
        print('.',end='',flush=True)
        pdelta = d2Step / a2Max

        # Find tIn based on ax2 requirement
        tIn = numpy.sqrt(6) * numpy.sqrt(d2In / a2Max) # min required for ax2

        v1Diff_max = a1Out * tIn / 2 # max achievable within tIn

        v1Diff_HLM = v1Scan * v1TolRatio
        if v1Diff_max > v1Diff_HLM: # too much
            v1Diff_max = v1Diff_HLM

        if v1Diff > v1Diff_max:  # apply by limiting v1Diff down
            # Exceeding VTol, then use up VTol and reduce Inner Skip
            v1Diff = v1Diff_max

        # (rscn_WellCond)
        # based on VAdd, tIUSkip behaviour changes: between the limits, tIn
        # has an omptimal value which optimizes the tOvrHd. with VAdd higher
        # than Hi, tIn willincrease tOvrHd, so it ashall not be
        # increased. with VAdd lower than min, tIn can be increased.
        # Therefore, this tIn gives a high limit for

        v1Out = v1Scan - v1Diff

        alpha = (a1Out * tIn + 2 * v1Diff)
        tOut = (2 * v1Out * tIn) / alpha



        # Find (tEdge/2) for ax2

        ptau = (numpy.sqrt(tIn ** 2 + 24 * pdelta) - 3 * tIn) / 8                        #(rscn_ptau_ax2)
        # ptau is monotonically decreasing to tIn right?
        tEdge = 4 * ptau - 2 * tOut

        # to check how the total time is converging towards its minimum

        tOvrHd = 2*(tOut + (tEdge / 2) + (v1Diff * tIn) / (3 * v1Scan))
        pdeltatOv = tOvrHd - tOvrHdOld


        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # ax2 is SLOWER than ax1... try optimising d1Out
        if (tEdge > tSolverMin):
            # if VTol is used to make ax1 return faster, then
            if (v1Diff > 0):
                # Use LESS inner decceleration on Fly axis to decrease d1Out
                # slowing down the ax1,

                # eat up part of (tEdge/2)
                pdeltaT = (tEdge / 2) / 3
                tOutdum = tOut - pdeltaT
                # decrease VAdd
                # (rscn_VAdd_ax1)[v1Diff=
                dum = -(tIn * (a1Out * tOutdum - 2 * v1Scan)) / (2 * (tOutdum + tIn))
                if (dum < 0):
                    v1Diff = 0
                else:
                    if (dum < v1Diff_max):
                        v1Diff = dum

                #         else
                #             % Thats it: leave (tEdge/2)
                #             (tEdge/2)=(tEdge/2)#
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        if (tEdge < -tSolverMin):
            # ax2 is faster than ax1: try making ax1 faster by adding v1Diff
            if (v1Diff < v1Diff_max) and firstPass:
                pdeltaT = (tEdge / 2) / 1.1
                tOutdum = tOut + pdeltaT                                                # eat up part of (tEdge/2) into tOut

                firstPass = 0

                # (rscn_VAdd_ax1)[v1Diff=
                dum = -(tIn * (a1Out * tOutdum - 2 * v1Scan)) / (2 * (tOutdum + tIn))

                if dum < v1Diff_max:
                    v1Diff = dum
                else:
                    v1Diff = v1Diff_max
            else:
                # -------------
                # else, decrease d2In distance/time :
                # because it actually makes ax1 faster, if v1Diff>0

                pdeltaT = (tEdge / 2) / 3

                dum = v1Diff / a1Out * 2
                if tInLLM > dum:
                    dum = tInLLM

                if ((tIn + pdeltaT) > dum):
                    dum = tIn + pdeltaT                                                            # eat up part of (tEdge/2) into tOut
                    d2In = dum * dum * a2Max / 6
                else:
                    # ax2 is too fast but ax1 can not be made faster by adding VDiff, so reduce a2
                    # CHECK THIS FORMULA

                    dum = abs((6 * d2Step) / ((tOut + tIn) * (4 * tOut + 2 * tIn + 3 * 0 * tEdge)))
                    if dum < a2Max :
                        a2Max = dum
                    else:
                        a2Max = a2Max * 0.9

                    # Now retry increasing v1Diff
                    #firstPass = 1

                    d2In = d2In / a2Max * a2Max

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