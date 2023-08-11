%% SimLoop
% Sorry no actual help!



%%

clear global
clear variables
clc
close all
diary off

diaryTag = input('Tag for diary:','s')
diaryFileName = 'nrSimLoop_lastDiary';   %must be checked out! c:\P4C\tec\mc\pmacRascan\trunk\mfiles\nrSimLoop_lastDiary
copyfile(diaryFileName,'nrSimLoop_b4lastDiary','f');
delete(diaryFileName)
diary(diaryFileName)
diary on

%% interface definition

%Scan instance inputs
global d1Centre d2Start lineN lineEnd     
% Status & info variables
global devError tLine tOvrHd 
%CS parameters. Not in solver
global ax1HLMUsr ax1LLMUsr ax2HLMUsr ax2LLMUsr ax1OffUsr ax2OffUsr pE1Res pE2Res
% Trajectory solver essential inputs
global tInLLM d1InHLM d1OutHDM d1Span d2TolRatio d2Step v1ScanHLM v1ScanLLM a1HLM a2HLM j1HLM j2HLM 
% Trajectory solver essential outputs
global tIn tOut tEdge d1Out v1Scan v1Diff a1Out a2Max

% variables set by Rascan motion program
global tMid tHLM lineSubEnd 

% Rascan Indent
global d1IndL d1IndR d1IndHLM d1IndLReq d1IndRReq

global SampleTime ax1tsMotion ax2tsMotion  d1IndRFun d1IndLFun m_IndentationOn %simulation variables

%% basic settings

SampleTime=0.0002; %ms

nrSetInputs

% Overrides


%d1Span=1%100*d2Step/a2HLM*v1Scan/20 %conservatively fixed
%d1InHLM=0.1% limits the use of inner skip 


% (tEdge>>tHLM) and GROWING by decreasing tOut because of a FUNDAMENTAL issue:
% One ESkip move means ax2AESkip has to remain 0 (because it needs to be averaged out)
% Which means ax2 can only gain velocity during IOSkip. This in turn limits
% the optimization massively   ---- FIXED!!!


Eval1Str='tLine';
Eval2Str='d1Out';
Eval3Str='tOvrHd';


%Test vectors: Set the test vectors

% Param1Str='d1InHLM';
% Param1 = [1,2,4,10]*0.05*d1Span;
% Param2Str='d2TolRatio';
% Param2 = [1,5,10]*0.05;
% % Param2Str='v1TolRatio';
% % Param2 = [0,1/4/1/3,1/2];

pE1Res=0.0001; pE2Res=0.00002;


% v1Scan=8;
% Param1Str='d1InHL M';
% Param1 = [0.05,0.5]*d1Span;
% % Param2Str='a2HLM';
% % Param2 = [0.1,0.5,1];
% Param2Str='v1Scan';
% Param2 = [2, 8];
% a1HLM=300; d2Step=0.004; a2HLM=1; v1TolRatio=0.8; d1Span=1;

timeScaleFactor = 1;

%CIL test
Param2Str='v1ScanHLM';
Param2 = [5, 10];
% Param2Str='a2HLM';
% Param2 = [0.1,0.5,1];
Param1Str='a1HLM';
Param1 = [30, 300];
d1Centre=0;d2Start=0; a1HLM=300/timeScaleFactor^2; d2Step=0.002; a2HLM=1/timeScaleFactor^2; 
d2TolRatio=0.25; d1Span=1.25; d1InHLM=0.2; d1IndHLM=0.2;
v1TolRatio=0.5;
d1OutHDM = 0.02;
v1ScanHLM = 5;
j1HLM = 20000;

% Save original conditions
eval(['orig_param1 = ',Param1Str,';']);
eval(['orig_param2 = ',Param2Str,';']);

m_showingIsOn=1;
m_IndentationOn=0
lineEnd=4;

%% Main loops

Loop1N = length(Param1);
Loop2N = length(Param2);

m_Eval1 = NaN(Loop1N,Loop2N);
m_Eval2 = NaN(Loop1N,Loop2N);
m_Eval3 = NaN(Loop1N,Loop2N);

warning('off','all')
f4legend='';

figure(4)
title('Optimized RASCAN');xlabel('Fly axis');ylabel('Step axis');grid on;hold on 

for i2 = 1: Loop2N
    for i1 = 1: Loop1N
        eval([Param2Str,'=Param2(i2);'])
        eval([Param1Str,'=Param1(i1);'])
        
        v1ScanLLM = (1 - v1TolRatio)*v1ScanHLM;        
        %% set and print lables before 
        rundumStr=sprintf(['\n==== Case ==== < R%1i-%1i > ',Param2Str,'=%3.4f ',Param1Str,'=%3.4f \n'],i2,i1,Param2(i2),Param1(i1));
        fprintf(rundumStr)
        fprintf('\n')
        varNames = cellstr([{'tInLLM'},'d1OutHDM','d1Span','d2TolRatio','d2Step','v1ScanHLM','v1ScanLLM','a1HLM','a2HLM','j1HLM','j2HLM']);
        %varVals = num2cell([tInLLM,d1OutHDM,d1Span,d2TolRatio,d2Step,v1ScanHLM,v1ScanLLM,a1HLM,a2HLM,j1HLM,j2HLM]);
        for i = 1 : length(varNames)
            dumVarName = cell2mat(varNames(i));
            dumV = eval(dumVarName);
            fprintf([dumVarName,'=%3.4f '],dumV)
        end
        
        %disp(cell2table(varVals,'VariableNames',varNames))
        
        %% pre validate the case
        devError = 'None';
        errCond = 'v1ScanLLM*tInLLM > d1InHLM';
        if eval(errCond)
            fprintf('\nCase is inconsistent: (')
            fprintf(errCond)
            fprintf(') !! \n')
            devError = errCond;
        end
        
        %% run solver
        if strcmp(devError,'None')
            fprintf('\n-- Solver starting... \n')
            nrRunDbl;
        
            %% set and print lables after

            fprintf('\n-- Solver finished.')
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
        %% run motion program        
        if strcmp(devError,'Solved')
            fprintf('\n-- Simulation starting... \n')
            lineN=0;
            
            ndDynMask_Prog;

            %%

            ax1tsMotion.Name='Ax1Motion';
            ax1tsMotion.TimeInfo.Units='seconds';
            ax1tsMotion.TreatNaNasMissing = true;
            ax2tsMotion.Name='Ax2Motion';
            ax2tsMotion.TimeInfo.Units='seconds';
            ax2tsMotion.TreatNaNasMissing = true; 
            fprintf('\n-- Simulation finished.')

            %% setup presentation
            %fprintf('\n -- Presentation starting... \n')

            % to set the start of first line at time 0
            m_lineStartTime = 1*tIn+2*tOut+tEdge+0*(tMid+(lineSubEnd)*tHLM) -tOut;
            m_TotalTime=lineEnd*tLine;
            m_TotalOvrHd=lineEnd*(tOvrHd);
            m_TotalScanTime=m_TotalTime-m_TotalOvrHd;
            m_OvrHdRatio=m_TotalOvrHd/m_TotalScanTime;
            fprintf(['Total=%3.4f[s]', ' Ovr=%3.4f[s]'],m_TotalTime,m_TotalOvrHd)

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
                    f4legend=[f4legend;sprintf('R%2i-%2i : %1.1es %1.1emm',i2,i1,tLine,d1Out)];
                    legend(f4legend);
                    fig=gcf;
                    fig.OuterPosition=[10,100,1000,1000];
                    drawnow;


                    m_Eval1(i1,i2) = eval(Eval1Str);
                    m_Eval2(i1,i2) = eval(Eval2Str);
                    m_Eval3(i1,i2) = eval(Eval3Str);

                    if m_showingIsOn

                        figure(1)
                        plot(ax1tsMotion.time, ax1tsMotion.Data(:,1))
                        title('Rascan axis 1');xlabel('time[s]');ylabel('Position [EGU]');grid off; 
                        nPlotVertLines(timeLines,timeLables);

                        figure(5)
                        plot(ax2tsMotion.time, ax2tsMotion.Data(:,1));%hold on                
                        title('Rascan axis 2');xlabel('time[s]');ylabel('Position [EGU]');grid on; 
                        nPlotVertLines(timeLines,timeLables);

                        figure(2)
                        plot(ax1tsMotion.time, ax1tsMotion.Data(:,2));%hold on
                        title('Rascan axis 1');xlabel('time[s]');ylabel('Velocity [EGU/s]');grid on; 
                        nPlotVertLines(timeLines,timeLables);

                        figure(6)
                        plot(ax2tsMotion.time, ax2tsMotion.Data(:,2));%hold on                 
                        title('Rascan axis 2');xlabel('time[s]');ylabel('Velocity [EGU/s]');grid on;
                        nPlotVertLines(timeLines,timeLables);

                        figure(3)
                        plot(ax1tsMotion.time, ax1tsMotion.Data(:,3));%hold on
                        title('Rascan axis 1');xlabel('time[s]');ylabel('Acceleration [EGU/s^2]');grid on;
                        nPlotVertLines(timeLines,timeLables);

                        figure(7)
                        plot(ax2tsMotion.time, ax2tsMotion.Data(:,3));%hold on                 
                        title('Rascan axis 2');xlabel('time[s]');ylabel('Acceleration [EGU/s^2]');grid on;
                        nPlotVertLines(timeLines,timeLables);                    

                        pause
                    end
                end
            end
        end
        fprintf('\n=========================================== \n')
    end
end

%% plot

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


if exist('ax1tsMotion','var') && isempty(devError)
    if (ax1tsMotion.Length > 1)
        figure
        plot(ax1tsMotion.time, [ax1tsMotion.Data ax2tsMotion.Data])
    end
end


if max([size(Param1),size(Param2)])>1
    myMeshResult(Param2,Param1,m_Eval1,Param2Str,Param1Str,Eval1Str);
    myMeshResult(Param2,Param1,m_Eval2,Param2Str,Param1Str,Eval2Str);
    myMeshResult(Param2,Param1,m_Eval3,Param2Str,Param1Str,Eval3Str);
else
    fprintf('something went wrong here !!!')
end


diary off

thisDiaryFileName = ['_out\','nrSimLoop_',diaryTag,'_' , datestr(datetime,'_yymmdd_HHMM') , '.out'];
copyfile(diaryFileName,thisDiaryFileName);