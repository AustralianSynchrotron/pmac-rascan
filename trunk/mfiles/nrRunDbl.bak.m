function []=nrRunDbl()


% Double Optimization solver

% solving rascan dblOpt 
% solution for rascan
% this routine targets minimising turnaround time: tIn + (tEdge/2)
% with these constraints:

% accelerations: ax1 and ax2 trajectoeries maximum acceleration are within the limits: a2HLM

% ax1 velocity: at v1Scan within tolerance v1TolRatio, while inside the image
% range specified by d1Span 

% ax2 position: pixel range defined by d2Step within tolerance
% d2TolRatio while inside the image range specified by d1Span 

% NOTE: turaround overscan d1Out might not be minimized

%% interface definition

global d1Centre d2Start lineEnd tInLLM a1HLM v1Scan d2Step d1Span j1HLM j2HLM %Rascan gsu
global tMid tOut lineN lineSubEnd devError tEdge tIn v1Diff a2Max d1Out tOvrHd tLine a1Out % Rascan gs
global ax1HLMUsr ax1LLMUsr ax2HLMUsr ax2LLMUsr ax1OffUsr ax2OffUsr pE1Res pE2Res %CS
global d2TolRatio v1TolRatio a2HLM d1InHLM %Rascan Solver
global d1IndL d1IndR d1IndHLM d1IndLReq d1IndRReq% Rascan Indent
global SampleTime ax1tsMotion ax2tsMotion tHLM d1IndRFun d1IndLFun m_IndentationOn %simulation

%% Solver parameters initial point and learning Factor

iterMax=100;
erTol=1e-6;
tSolverMin=1e-4*2;


%% ---------------- BEGIN BLOCK INIT

%>>>> Optimisation is based on an iterative procedure: 
% An initial point is chosen to maximize ax1 without v1Diff
% at first pass, if ax1 is too slow, then a maximum possible v1Diff is set
% after this, target is to have a negative tEdge which can be gradually mved back to 0

%parameters are v1Diff, and

a1Max=a1HLM;
a1MaxOld=a1Max;

if (d2Step> 0)
    a2Max=abs(a2HLM);
else
    a2Max=-abs(a2HLM);
end
a2MaxOld=a2Max;

a1Out=a1Max;
v1Out=v1Scan;
v1Diff_max = v1Scan*v1TolRatio;
v1Diff_min = 0;
v1Diff = v1Diff_min;
%>>>> Here we want to set d1In, d2In and ultimately tIn at their max. 
% iterative algorithm should need to decrease or maintain these.

% establish d1In at its max
d1In_max = d1Span/2*0.95;
if (d1In_max > d1InHLM) %&& (v1TolRatio>0)
    d1In_max = d1InHLM;
end;

% start at minimum
tIn_max = tInLLM;

% set tIn to satisfy d1In: conservative setting diregarding the v1Diff variations...
dum = d1In_max/(v1Scan-v1Diff);
if dum > tIn_max
    tIn_max = dum;
end

%Conservative assumption: regardless of VTol
% Now set d2In at its max possible with tIn and a2Max... !!!

d2In_max = (a2Max*tIn_max*tIn_max)/6;
dum=d2Step*d2TolRatio;
if d2In_max >  dum; % start at highest
    d2In_max = dum;
    tIn_max = sqrt(6)*sqrt(d2In_max/a2Max);
end

tIn = tIn_max;
tInOld = tIn;

d1In = d1In_max;

d2In = d2In_max;
d2InOld=d2In;

myTextFormat = 'Initial point:tIn=%3.4f d1In=%3.5f d2In=%3.5f v1Diff=%3.4f \n';
fprintf(myTextFormat,tIn,d1In,d2In,v1Diff);
% d2In is good only if its not making ax2 too fast, which will be indicated by a (tEdge/2) < 0

tOut=0;
tOutOld=tOut;

tEdge=0;
tEdgeOld=tEdge;

tOvrHd=10000;
tOvrHdOld=tOvrHd;

i=1;
er=100;
%---------------- END BLOCK INIT
% numerically solving double optimization problem
erLog=zeros(iterMax,5);

firstPass=1;
fprintf('solving')
%% ---------------- BEGIN BLOCK ITER
while (i < iterMax) && (er > erTol)
    fprintf('.') 
    pdelta=d2Step/a2Max;
    
    % Find tIn based on ax2 requirement
    tIn=sqrt(6)*sqrt(d2In/a2Max);
    
	v1Diff_max=a1Out*tIn/2;
    
    dum=v1Scan*v1TolRatio;
    if v1Diff_max > dum 
        v1Diff_max=dum;
    end

    
    %% Jerk calculations
    a = a1Out*tIn;
    b = v1Scan+a;
    if (b^2-2*a^2 > 0)
        % v1Diff whih makes j1Out=j1In
        v1Diff_equal = (sqrt(b^2-2*a^2)+b)/4;
    else
        % doesnt exist
        v1Diff_equal  = nan;
    end
    % v1Diff wich makes j1Out minimal
    v1Diff_opt = a/2; 
    j1HLM_opt = (2*v1Diff_opt)/tIn^2;    
    %j1HLM = 10*j1HLM_opt;
    
    v1DiffByJerk=j1HLM*tIn*tIn/2;
    if v1Diff_max > v1DiffByJerk 
        v1Diff_max=v1DiffByJerk;
        fprintf(' v1Diff_max -> f_j1LLM ')
    end
            
    if v1Diff > v1Diff_max;
        % Exceeding VTol, then use up VTol and reduce Inner Skip
        v1Diff = v1Diff_max;
        fprintf(' v1Diff -> v1Diff_max ')
    end
    v1Diff_min = (2*j1HLM*tIn*v1Scan-a1Out^2*tIn)/(2*j1HLM*tIn-2*a1Out);
    if v1Diff_min > v1Diff_max
        %v1Diff_min = 0;
        fprintf(' v1Diff_minmax! ')
    end
    
    
    %% dependent variables calculations
    
    %(rscn_WellCond)
    % based on VAdd, tIUSkip behaviour changes: between the limits, tIn
    % has an omptimal value which optimizes the tOvrHd. with VAdd higher
    % than Hi, tIn willincrease tOvrHd, so it ashall not be
    % increased. with VAdd lower than min, tIn can be increased.
    % Therefore, this tIn gives a high limit for 
 
    v1Out=v1Scan-v1Diff;

    alpha=(a1Out*tIn+2*v1Diff);
    tOut=(2*v1Out*tIn)/alpha;
    
    
    
    %Find (tEdge/2) for ax2

    ptau=(sqrt(tIn^2+24*pdelta)-3*tIn)/8; %(rscn_ptau_ax2) 
    %ptau is monotonically decreasing to tIn right?
    tEdge=4*ptau-2*tOut;
   
    % to check how the total time is converginhg towards its minimum

    tOvrHd = 2*(tOut+(tEdge/2)+(v1Diff*tIn)/(3*v1Scan));
    tLine = d1Span/v1Scan+tOvrHd;
    %pdeltatOv=tOvrHd-tOvrHdOld;
    
    myTextFormat = 'v1Diff %3.4f < %3.4f (%3.4f) <%3.4f j1HLM_mid=%3.4f\n';
    fprintf(myTextFormat,v1Diff_min,v1Diff,v1Diff_opt,v1Diff_max,j1HLM_opt);    
%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    %% case ax1 turn-round is quicker
    if (tEdge > tSolverMin)
    % ax2 is SLOWER than ax1... try optimising d1Out
        %? ax2 is too slow, 
        % :decrease currently positive tEdge to minimum
        
        
        % if VTol is used to make ax1 return faster, then
        if (v1Diff > 0) 
            %? ...
            % and v1Diff decceleration is used. 
                
            
            
            % Use LESS inner decceleration on Fly axis to decrease d1Out
            % slowing down ax1, 
            
            % eat up part of (tEdge/2)
            pdeltaT=(tEdge/2)/3;
            tOutdum=tOut-pdeltaT;
            % decrease VAdd
            %(rscn_VAdd_ax1)[v1Diff=
            dum=-(tIn*(a1Out*tOutdum-2*v1Scan))/(2*(tOutdum+tIn));            
            if (dum < 0)
               v1Diff = 0;
               fprintf('v1Diff=0')
            else
                if (dum < v1Diff_max)
                    v1Diff=dum; 
                    fprintf('v1Diff=<')
                end
            end
        
%         else
%             % Thats it: leave (tEdge/2)
%             (tEdge/2)=(tEdge/2);
        end
    end   
    %^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    
    %^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    
    %% case ax2 turn-around is quicker
    if (tEdge < -tSolverMin) 
        %% ax2 is too fast, need to increase currently negative tEdge to 0 
        %>>> ax2 is FASTER than ax1% OSkip SHALL be reduced:
        %? ... 
        % and ax1 can't be made faster using v1Diff or it is done
        % once.
        % check options...

        if (v1Diff < v1Diff_max) && (v1Diff ~= v1Diff_opt)
            %% optimise velocities
            pdeltaT=(tEdge/2)/1.1;
            % find the actual limit for tIn to respect ax1 deceleration
            %as well as LLM

            tOutdum=tOut+pdeltaT;                
            dum=-(tIn*(a1Out*tOutdum-2*v1Scan))/(2*(tOutdum+tIn));

            if dum < v1Diff_max
                v1Diff=dum;
            else
                v1Diff=v1Diff_max;
            end
            fprintf('v1Diff=opt')            
        else
            %% reduce a2Max to get rid off tEdge
            %? ax2 is too fast, and ax1 can't be made faster using v1Diff. 
            % and tIn can't be further decreased without compromising v1Diff.
            %: Decrease a2Max to increase currently negative tEdge to 0
            dumFactor = 1/8;
            dum=abs((6*d2Step)/((tOut+tIn)*(4*tOut+2*tIn+3*dumFactor*tEdge)));
            if dum < a2Max
                d2In = d2In *dum /a2Max;
                a2Max = dum;
                fprintf('a2Max=<')
            else
                dum = a2Max * 0.9;
                d2In = d2In *dum /a2Max;
            a2Max=dum;
                fprintf('a2Max/')
            end

            % Inorder for the change to be retained, d2In has to change
            % accordingly. Otherwise tIn will be calculated based on
            % reduced a2Max and nullify its effect
            %d2In=d2In/a2Max*a2Max;
        end
    end    
    %^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    %^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    %^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  
    %% assert convergence
    erLog(i,1)=tOut-tOutOld; tOutOld=tOut;
    erLog(i,2)=tIn-tInOld; tInOld=tIn;
    erLog(i,3)=(tEdge/2)-(tEdgeOld/2); tEdgeOld=tEdge;   
    erLog(i,4)=a2Max-a2MaxOld; a2MaxOld=a2Max;
    erLog(i,5)=tOvrHd-tOvrHdOld; tOvrHdOld=tOvrHd;
    
    er=max(abs(erLog(i,:)));
    i=i+1;
end
%---------------- END BLOCK ITER

%% 
if i==iterMax
    myTextFormat = '\n   !!!!!!!!! Solution NOT FOUND ... iter=%4.0f error=%4.3e\n';
    fprintf(myTextFormat,i,er);
    display(erLog(i-min(i-1,3):i-1,:));
    lineSubEnd=0;
    devError='   Solver Did Not Converge';
    d1Out=0;
    m_lineStartTime=0;
    tOvrHd=0;
    return
end

if i < iterMax 
    assert(abs(v1Scan-v1Out-v1Diff)<1e-10,'ERROR');
    myTextFormat = '\n %4.0f iters error=%4.3e\n';
    fprintf(myTextFormat,i,er);
%% setup scan shaping via indentation


    d1IndRFun=[0,-0.1,   0.1,  -0.2,-0.1,-0.2,-0.3];
    d1IndLFun= [0,  0.1,-0.1,  0.2,-0.1,-0.2,-0.3];
    
%     d1IndLFun=(cos([0:lineEnd/2-2]*pi/(lineEnd/4))-1)*d1Span;
%     d1IndRFun=d1IndLFun;
%     
%     d1IndLFun=(-cos([0:lineEnd/2-2]*pi/(lineEnd/4))+1)*d1Span;
%     d1IndRFun=-d1IndLFun;
    
%% run motion program        
    lineN=0;
    nrRascan_Prog;
    
    %%

    ax1tsMotion.Name='Ax1Motion';
    ax1tsMotion.TimeInfo.Units='seconds';
    ax1tsMotion.TreatNaNasMissing = true;
    ax2tsMotion.Name='Ax2Motion';
    ax2tsMotion.TimeInfo.Units='seconds';
    ax2tsMotion.TreatNaNasMissing = true; 
    
end