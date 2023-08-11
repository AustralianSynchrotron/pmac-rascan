function [NextPointA, tsMotion]=nrIncPVT(tsMotion,PointA,PointBInc,SampleTime,PosRes, vCycles)

% PointX = [AbsPos,AbsVel,IntervalTime]
    if (PointBInc(3) <= SampleTime)
        NextPointA=PointA;
        return
    end
    PointBInc=PointBInc+PointA.*[1,0,1]; %incremental position to absolute
    PointB=PointBInc;

    tsMotion = append(tsMotion, nMotion_PVT( PointA , PointB , SampleTime, PosRes, vCycles ));
    NextPointA =PointB;

end