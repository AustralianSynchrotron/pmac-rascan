function [ tsObject ] = nMotion_PVT( PointA, PointB, SampleTime,PosRes, vCycles)
%Motion_PVT creates a profile of target positions and velocities based on
%PVT rules
%   Detailed explanation goes here



% Velocity quantization vactor
VelRes = PosRes/(SampleTime*vCycles);
AccRes = VelRes/(SampleTime*vCycles);


% Motion starts at point A and end at point B
PosA = PointA(1);%round(PointA(1)/eres)*eres;
VelA = PointA(2);% round(PointA(2)/eres*32*SampleTime)*eres/32/SampleTime;
TimeA= PointA(3);
PosB = PointB(1);%round(PointB(1)/eres)*eres;
VelB = PointB(2);%round(PointB(2)/eres*32*SampleTime)*eres/32/SampleTime;
TimeB= PointB(3);

% Base vactors
TimeVec0=[0:SampleTime:TimeB-TimeA]';
VecLen=length(TimeVec0);
PosVec = zeros(size(TimeVec0));
VelVec=PosVec;

% Average velocity
PosDelta= PosB - PosA;
TimeDelta = TimeB - TimeA;
VelAvg = PosDelta/TimeDelta;

% PVT is a linear acceleration motion. Acceleration can be calculated at
% the start and end points:
AccA = (+6*VelAvg - 2*VelB - 4*VelA)/TimeDelta;
AccB = (-6*VelAvg + 4*VelB + 2*VelA)/TimeDelta;
AccVec=linspace(AccA,AccB,VecLen)';
AccDelta = AccB-AccA;

% For the known linear acceleration, velocity and position can be calculated:
VelVec=VelA+ AccA.*TimeVec0 + 1/2*AccDelta/TimeDelta.*TimeVec0.^2;
PosVec = PosA + VelA.*TimeVec0 + 1/2*AccA.*TimeVec0.^2 + 1/6*AccDelta/TimeDelta.*TimeVec0.^3;
% Time vector adjusted
TimeVec0=TimeVec0+TimeA;
%plot(TimeVec0, [PosVec,VelVec])
v=[PosVec, VelVec, AccVec];
% 
% digitVector= ones(size(TimeVec0))*[PosRes, VelRes, AccRes];

% v=round([(v + digitNoiseVector)./digitVector]).*digitVector;

digitFlicker = 2;
digitNoise = randn([size(v,1),1])/2*digitFlicker;

vecRes = [PosRes, VelRes, AccRes];
for i = 1: size(v,2);
    v(:,i) = round((v(:,i)) /vecRes(i)+digitNoise)*vecRes(i);
end

tsObject = timeseries(v, TimeVec0);

end

