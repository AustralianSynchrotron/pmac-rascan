%% SimLoop
% Simulating RASCAN2 solver

%% Rascan Variable Naming Convention:

% first leter denotes physical dimension of the variable: 
% (d)istance[mm], % (v)elocity[mm/s], (a)cceleration[mm/s^2], (j)erk[mm/s^3], (t)ime[s], (m)ass[kg],
% tor(q)ue[Nm], (f)orce[N]
% following "digit" is 1: fast axis, 2: slow axis
% third: 
%   In: inner turnaround region
%   Out: outer turnaround region
%   Scan: mid scan region
% forth describer:
%   LLM: Lower Limit
%   HLM: Higher Limkit
%   Line: Scan line, full translation of the fast axis in the scan region
%   and on turnaround
%   Ovr: Overal
%   HDM: High desirable Limit
%   RES: resolution


%% Presets and Interface Definition

clear global
clear variables
clc
close all
diary off

newcolors = [0.83 0.14 0.14
             1.00 0.54 0.00
             0.47 0.25 0.80];
             %0.25 0.80 0.54];
         

newcolors = [newcolors ; transpose(newcolors)];
         
         
diaryTag = ''; %input('Tag for diary:','s')
diaryFileName = 'nrSimLoop_lastDiary';   %must be checked out! c:\P4C\tec\mc\pmacRascan\trunk\mfiles\nrSimLoop_lastDiary
copyfile(diaryFileName,'nrSimLoop_b4lastDiary','f');
delete(diaryFileName)
diary(diaryFileName)
diary on

% interface definition

%Scan instance inputs
global d1Centre d2Start lineN lineEnd     
% Status & info variables
global devError tLine tOvrHd 
%CS parameters. Not in solver
global ax1HLMUsr ax1LLMUsr ax2HLMUsr ax2LLMUsr ax1OffUsr ax2OffUsr pE1Res pE2Res vCycles
% Trajectory solver essential inputs
global tInLLM d1InHLM d1OutHDM d1Span d2TolRatio d2Step v1ScanHLM v1ScanLLM a1HLM a2HLM j1HLM j2HLM 
% Trajectory solver essential outputs
global tIn tOut tEdge d1Out v1Scan v1Diff a1Out a2Max

% variables set by Rascan motion program
global tMid tHLM lineSubEnd 

% Rascan Indent
global d1IndL d1IndR d1IndHLM d1IndLReq d1IndRReq

global SampleTime ax1tsMotion ax2tsMotion  d1IndRFun d1IndLFun m_IndentationOn %simulation variables

% simulated jerk values
global j1In j1Out j2In j2Out j2Edge 

%% Positions and offsets (irrelevent to RASCAN solution)

EGU = 'mm';
pos_label = sprintf('Position [%s]',EGU);
vel_label = sprintf('Velocity [%s/s]',EGU);
acc_label = sprintf('Acceleration [%s/s^2]',EGU);

big_step_factor = 1; %0.1, 1 for Small or 10 for Large step

ax1OffUsr=0;
ax1LLMUsr=-10000;
ax1HLMUsr=10000;

ax2OffUsr=0;
ax2LLMUsr=-10000;
ax2HLMUsr=10000;
d1Centre=0;
d2Start=0; 

%% MEX Hardware Constraints
mm_2_meter = 1e-3;
timeScaleFactor = 1;

% Resolutions
SampleTime=0.0002; 
pE1Res=0.00002; pE2Res=0.00002;  
vCycles=32; % max servo cycles between velocity changes, this determines effective velocity and acceleration resolutions

v1Res = pE1Res/(SampleTime*vCycles)
a1Res = v1Res/(SampleTime*vCycles)

v2Res = pE2Res/(SampleTime*vCycles)
a2Res = v2Res/(SampleTime*vCycles)


% to determine the acceleration and Jerk limits:
MechDesignMargin = 0.1;

mSamplePlate = 0.129;
mKinematicMount = 0.351;
mYStageTravel = 0.509;
m1Total = mSamplePlate + mKinematicMount + mYStageTravel;
mOrthoBracket = 0.344;
mYStageFixed = 1.024;
mXStageTravel = mYStageTravel;
m2Total = mOrthoBracket + mYStageFixed + mXStageTravel + m1Total;
mXStageFixed = mYStageFixed;
mXBracket = 1.5;
mZStageTravel = 0.509;
m3Total = mXStageFixed + mXBracket + mZStageTravel + m2Total;

% adding design margin
m1HLM = m1Total * (1 + MechDesignMargin);
% adding design margin
m2HLM = m2Total * (1 + MechDesignMargin);

f1MechLimit = 3;
f1MechHLM = f1MechLimit/4; % Nm
f2MechHLM = f1MechHLM;

% it takes this much time for the motor to build-up full emf
t1ForceRampup = 100e-3; 
% in our application, we can tolerate about this much of build-up time
t1ActualRampup = 5e-3; 

a1SteadyState = f1MechHLM/m1HLM / mm_2_meter / timeScaleFactor^2;
a2SteadyState = f2MechHLM/m2HLM / mm_2_meter / timeScaleFactor^2;

j1HLM = a1SteadyState / t1ForceRampup;
j2HLM = a2SteadyState / t1ForceRampup;

a1HLM = j1HLM * t1ActualRampup;
a2HLM = j2HLM * t1ActualRampup;



%% Solver Optimisation Parameters

tInLLM=t1ActualRampup/2;
tMidLLM=0.002;
tHLM=4;

d2TolRatio=0.25;
d1InHLM=0.02 * big_step_factor; 
d1IndHLM=0.02 * big_step_factor;
d1OutHDM=0.02 * big_step_factor;

%% MEX Scan Parameters
d1Span=0.1 * big_step_factor;
d2Step=0.0008; 
v1ScanHLM = 1;
v1TolRatio = 0.5;
lineEnd=2;

%% Simulation Settings

Eval1Str='tLine';
Eval2Str='d1Out';
Eval3Str='tOvrHd';

Param2Str='v1ScanHLM';
Param2 = [1, 4]*v1ScanHLM*big_step_factor^2;

Param1Str='d2Step';
Param1 = [(1-5/8), 1, 4]*d2Step* max(1,big_step_factor^2);

% Save original conditions
eval(['orig_param1 = ',Param1Str,';']);
eval(['orig_param2 = ',Param2Str,';']);

m_showingIsOn=1;
m_IndentationOn=0;


%% Simulation Loop

Loop1N = length(Param1);
Loop2N = length(Param2);

m_Eval1 = NaN(Loop1N,Loop2N);
m_Eval2 = NaN(Loop1N,Loop2N);
m_Eval3 = NaN(Loop1N,Loop2N);

warning('off','all')
f4legend='';

figure(4)
title('Optimized RASCANs: legend shows Line Time (tLine) and Overscan (d1Out) for each run');xlabel('Fly axis [mm]');ylabel('Step axis [mm]');grid on;hold on 
set(gca, 'ColorOrder', newcolors, 'NextPlot', 'replacechildren');

for i2 = 1: Loop2N
    for i1 = 1: Loop1N
        %% Solver
        eval([Param2Str,'=Param2(i2);'])
        eval([Param1Str,'=Param1(i1);'])
        
        v1ScanLLM = (1 - v1TolRatio)*v1ScanHLM;        
        % set and print lables before 
        rundumStr=sprintf(['run < %1i-%1i > ',Param2Str,' = %3.4f, ',Param1Str,' = %3.4f \n'],i2,i1,Param2(i2),Param1(i1));
        
        fprintf(['\n-- Simulation \nfor ',rundumStr, '\n'])
        
        varNames = cellstr([{'tInLLM'},'d1OutHDM','d1Span','d2TolRatio','d2Step','v1ScanHLM','v1ScanLLM','a1HLM','a2HLM','j1HLM','j2HLM']);
        
        for i = 1 : length(varNames)
            dumVarName = cell2mat(varNames(i));
            dumV = eval(dumVarName);
            fprintf([dumVarName,'=%3.4f '],dumV)
            if mod(i,5)==0
                fprintf('\n')
            end
        end
        
        % pre-validate the case
        devError = 'None';
        errCond = 'v1ScanLLM*tInLLM > d1InHLM';
        if eval(errCond)
            fprintf('\nCase is inconsistent: (')
            fprintf(errCond)
            fprintf(') !! \n')
            devError = errCond;
        end
        
        if strcmp(devError,'None')
            
            fprintf('\n-- Solver starting...\n')
            
            % this script finds the optimised RASCAN solution
            nrRunDbl;
        
            % set and print lables after

            fprintf('\n-- Solver finished. \n')
            varNames = cellstr([{'tIn'},'tOut','tEdge','d1Out','v1Scan','v1Diff','a1Out','a2Max']);
            for i = 1 : length(varNames)
                dumVarName = cell2mat(varNames(i));
                dumV = eval(dumVarName);
                fprintf([dumVarName,'=%3.4f '],dumV)
            end
            fprintf('\n')        
            varNames = cellstr([{'tLine'},'tOvrHd']);
            for i = 1 : length(varNames)
                dumVarName = cell2mat(varNames(i));
                dumV = eval(dumVarName);
                fprintf([dumVarName,'=%3.4f '],dumV)
            end        

            fprintf('\n')
        end
        %% Motion Program 

        if strcmp(devError,'Solved')
            fprintf(['\n-- Motion simulation starting \nfor ',rundumStr, '\n'])
            lineN=0;
            
            % this script simpulated a RASCAN scanner device
            nrRascan_Prog;
            
            ax1tsMotion.Name='Ax1Motion';
            ax1tsMotion.TimeInfo.Units='seconds';
            ax1tsMotion.TreatNaNasMissing = true;
            ax2tsMotion.Name='Ax2Motion';
            ax2tsMotion.TimeInfo.Units='seconds';
            ax2tsMotion.TreatNaNasMissing = true; 
            fprintf('\n-- Motion simulation finished. \n')

            %% Results
 
            % to set the start of first line at time 0
            m_lineStartTime = 1*tIn+2*tOut+tEdge+0*(tMid+(lineSubEnd)*tHLM) -tOut;
            m_TotalTime=lineEnd*tLine;
            m_TotalOvrHd=lineEnd*(tOvrHd);
            m_TotalScanTime=m_TotalTime-m_TotalOvrHd;
            m_OvrHdRatio=m_TotalOvrHd/m_TotalScanTime;
            
            
            fprintf('\n\n\n*************************************************************')
            fprintf(['\n-- Results \n  for ',rundumStr, '\n'])

            % fprintf('Total time = %3.4f[s]   Total Overhead = %3.4f[s] \n',m_TotalTime,m_TotalOvrHd)
            fprintf('Line time = %3.4f[s]   LineOverhead = %3.4f[s]  Overscan = %3.4f[mm] \n\n',tLine,tOvrHd,d1Out)

            fprintf(sprintf('Accelerations [%s/s^2] : \n',EGU))
            fprintf(check_print(a1Out,a1HLM, ''));
            fprintf(check_print(a2Max,a2HLM, ''));  
            fprintf('\n')            
            
            fprintf(sprintf('Jerks [%s/s^3] : \n',EGU))
            fprintf(check_print(j1In,j1HLM, ''));
            fprintf(check_print(j1Out,j1HLM, ''));  
            fprintf(check_print(j2In,j2HLM, ''));   
            fprintf(check_print(j2Out,j2HLM, ''));  
            fprintf(check_print(j2Edge,j2HLM, '')); 
            fprintf('\n')
            
            if m_lineStartTime ==0 % error
                break
            end

            timeLines=zeros(lineEnd*6+2,1);
            sectorLables={'Edge','Out','In','Mid','In','Out'};
            timeSectors=[tEdge,tOut,tIn,tMid+lineSubEnd*tHLM,tIn,tOut];
            timeLines(1)=tIn;
            for i=2:length(timeLines);
                j = mod(i-2,length(timeSectors))+1;
                timeLines(i)=timeLines(i-1)+timeSectors(j);
                timeLables(i-1)=sectorLables(j);
            end
            timeLables(length(timeLines))={' '};


            if exist('ax1tsMotion','var') && isempty(devError)
                if (ax1tsMotion.Length > 1)

                    % shift the time axis so that the return point is at t=0 (arbitrary,
                    % depends on lineEnd and lineN!!!!
                    ax1tsMotion.Time=ax1tsMotion.Time-m_lineStartTime;
                    ax2tsMotion.Time=ax1tsMotion.Time;
                    timeLines=timeLines-m_lineStartTime;

                    %resampTime=linspace(ax1tsMotion.Time(1),ax1tsMotion.Time(end),1000);
                    resampTime=ax1tsMotion.Time(1):0.001:ax1tsMotion.Time(end);

                    ax1tsMotion=resample(ax1tsMotion,resampTime);
                    ax2tsMotion=resample(ax2tsMotion,resampTime);
                    
                    figure(4)
                    simplot=plot(ax1tsMotion.Data(:,1),ax2tsMotion.Data(:,1),'.');hold on  
                    
                    f4legend=[f4legend; sprintf('R%2i-%2i : %1.1es %1.1emm',i2,i1,tLine,d1Out)];
                    legend(f4legend);
                    
                    l = length(ax1tsMotion.Data(:,1));
                    label_text=sprintf('%1i-%1i',i2,i1);
                    text(max(ax1tsMotion.Data(end,1)),max(ax2tsMotion.Data(end-round(l/500),1)),label_text)
                    fig=gcf;
                    set(gcf,'units','normalized')
                    set(gcf,'outerposition',[0.1 0.1 0.55 0.75])
                    %fig.OuterPosition=[10,100,1000,1000];
                    drawnow;


                    m_Eval1(i1,i2) = eval(Eval1Str);
                    m_Eval2(i1,i2) = eval(Eval2Str);
                    m_Eval3(i1,i2) = eval(Eval3Str);

                    if m_showingIsOn
                        
                        axis1Title = ['FLY axis - run (', label_text, ') '];
                        axis2Title = ['STEP axis - run (', label_text, ') '];
                        
                        figure(1)
                        plot(ax1tsMotion.time, ax1tsMotion.Data(:,1))
                        title([axis1Title, 'position']);xlabel('time[s]');ylabel(pos_label);grid off; 
                        nPlotVertLines(timeLines,timeLables);

                        figure(5)
                        plot(ax2tsMotion.time, ax2tsMotion.Data(:,1));%hold on                
                        title([axis2Title, 'position']);xlabel('time[s]');ylabel(pos_label);grid on; 
                        nPlotVertLines(timeLines,timeLables);

                        figure(2)
                        plot(ax1tsMotion.time, ax1tsMotion.Data(:,2));%hold on
                        title([axis1Title, 'velocity']);xlabel('time[s]');ylabel(vel_label);grid on; 
                        nPlotVertLines(timeLines,timeLables);

                        figure(6)
                        plot(ax2tsMotion.time, ax2tsMotion.Data(:,2));%hold on                 
                        title([axis2Title, 'velocity']);xlabel('time[s]');ylabel(vel_label);grid on;
                        nPlotVertLines(timeLines,timeLables);

                        figure(3)
                        plot(ax1tsMotion.time, ax1tsMotion.Data(:,3));%hold on
                        title([axis1Title, 'acceleration']);xlabel('time[s]');ylabel(acc_label);grid on;
                        nPlotVertLines(timeLines,timeLables);

                        figure(7)
                        plot(ax2tsMotion.time, ax2tsMotion.Data(:,3));%hold on                 
                        title([axis2Title, 'acceleration']);xlabel('time[s]');ylabel(acc_label);grid on;
                        nPlotVertLines(timeLines,timeLables);                    

                        % pause
                    end
                end
            end
        end
    end
end

%% Optimisation Plots

% figure(1)
% title('optimised raster scan');xlabel('time[s]');ylabel('Position[EGU]');grid on;hold off 
% figure(2)
% title('optimised raster scan');xlabel('time[s]');ylabel('Velocity[EGU/s]');grid on;hold off 
% figure(3)
% title('optimised raster scan');xlabel('time[s]');ylabel('Acceleration[EGU/s^2]');grid on;hold off 
% 
% figure(4)
% title('2D optimised raster scan');xlabel('Fly axis');ylabel('Step axis');grid on;hold off 
% %legend(f4legend);

% %% Last Run PVT plots
% if exist('ax1tsMotion','var') && isempty(devError)
%     if (ax1tsMotion.Length > 1)
%         figure
%         plot(ax1tsMotion.time, [ax1tsMotion.Data ax2tsMotion.Data])
%     end
% end

% %% Mesh plots
% 
% if max([size(Param1),size(Param2)])>1
%     myMeshResult(Param2,Param1,m_Eval1,Param2Str,Param1Str,Eval1Str);
%     myMeshResult(Param2,Param1,m_Eval2,Param2Str,Param1Str,Eval2Str);
%     myMeshResult(Param2,Param1,m_Eval3,Param2Str,Param1Str,Eval3Str);
% else
%     fprintf('something went wrong here !!!')
% end


diary off

thisDiaryFileName = ['_out\','nrSimLoop_',diaryTag,'_' , datestr(datetime,'_yymmdd_HHMM') , '.out'];
copyfile(diaryFileName,thisDiaryFileName);