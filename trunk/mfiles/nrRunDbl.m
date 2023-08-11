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

%% Solver parameters initial point and learning Factor

iterMax=100;
t_erTol=1e-6;
tSolverMin=1e-4*2;

v1_ErTol = 1e-5;
a1_ErTol = 1e-5;
d2_ErTol = 20e-9;

%% ---------------- BEGIN BLOCK INIT

%>>>> Optimisation is based on an iterative procedure: 
% An initial point is chosen to maximize ax1 without v1Diff
% at first pass, if ax1 is too slow, then a maximum possible v1Diff is set
% after this, target is to have a negative tEdge which can be gradually mved back to 0

%parameters are v1Diff, and

a1Max=a1HLM;

if (d2Step> 0)
    a2Max=abs(a2HLM);
else
    a2Max=-abs(a2HLM);
end
a2MaxOld=a2Max;

a1Out=a1Max;

v1Scan = v1ScanHLM;

v1Out=v1Scan;
v1Diff = 0;
v1Diff_d1In_min = 0;
%>>>> Here we want to set d1In, d2In and ultimately tIn at their max. 
% iterative algorithm should need to decrease or maintain these.

% establish d1In at its max
d1In_max = d1Span/2*0.95;
if (d1In_max > d1InHLM)
    d1In_max = d1InHLM;
end;

% start at minimum
tIn_max = tInLLM;

% set tIn to satisfy d1In: conservative setting diregarding the v1Diff variations...
dum = d1In_max/(v1ScanLLM);
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

myTextFormat = ' Initial point:tIn=%3.4f d1In=%3.5f d2In=%3.5f v1Diff=%3.4f \n';
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
thisIterChanges = '';
fprintf(' solving...\n')
%% ---------------- BEGIN BLOCK ITER
while (i < iterMax) && ((er > t_erTol) || ~isempty(thisIterChanges))
    fprintf('  ->%3i ',i) 

        
    pdelta=d2Step/a2Max;
    % Find tIn based on ax2 requirement
    tIn=sqrt(6)*sqrt(d2In/a2Max);


    %% Jerk calculations
   
    j1HLM_opt = a1Out/tIn;       
    
    v1Diff_j_max=j1HLM*tIn*tIn/2;
    %v1Diff_j_min = (tIn*sqrt(-8*j1HLM*v1Scan+j1HLM^2*tIn^2+4*a1Out^2)-j1HLM*tIn^2)/4;
    
    %% v1Diff limit calculations

	v1Diff_max=a1Out*tIn/2;
    % v1Diff wich makes j1Out minimal
    dum = v1Scan - v1ScanLLM;
    if v1Diff_max > dum
        v1Diff_max = dum;
    end    
    if v1Diff_max > v1Diff_j_max 
        v1Diff_max = v1Diff_j_max;
        fprintf(' v1Diff_j_max> ')
    end
    
    v1Diff_d1In_min = 3*v1Scan-(3*d1In_max)/tIn;
    % v1Diff min to keep d1In < d2Span/2*.95
  
    if v1Diff_d1In_min > v1Diff_max
        fprintf(' v1Diff_d1In_min! ')
    end    
    
    v1Scan_max = v1ScanHLM;
    dum = v1Diff/3 + d1In_max/tIn;
    if dum < 0 
        dum = 0;
    end    
    if v1Scan_max < dum
        v1Scan_max = dum;
    end
    
    %% v1Out optimal value to minimize tLine
    v1Out_opt = (sqrt(2)*sqrt(2*v1Diff+a1Out*tIn)*sqrt(tIn*v1Diff+2*d1Span)-4*sqrt(tIn)*v1Diff)/(4*sqrt(tIn));
    v1Diff_op_tLine = (2^(3/2)*sqrt(v1Scan)*sqrt(2*v1Scan+a1Out*tIn)-a1Out*tIn)/2;
    if v1Diff_op_tLine < 0 
        v1Diff_op_tLine = 0;
    end
    if v1Diff_op_tLine > v1Scan - v1ScanLLM
        %v1Diff_op_tLine = v1Scan - v1ScanLLM;
    end
        
    if v1Diff_op_tLine < v1Diff_max
        v1Diff_max = v1Diff_op_tLine;
    end    
    
    %% limit v1Diff
    if v1Diff > v1Diff_max;
        % Exceeding VTol, then use up VTol and reduce Inner Skip
        v1Diff = v1Diff_max;
        fprintf(' v1Diff:>_max ')
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
    d1Out=(4*(alpha-v1Diff)*v1Out*v1Out*tIn)/(3*alpha*alpha);
 
    %Find (tEdge/2) for ax2

    ptau=(sqrt(tIn^2+24*pdelta)-3*tIn)/8; %(rscn_ptau_ax2) 
    %ptau is monotonically decreasing to tIn right?
    tEdge=4*ptau-2*tOut;
   
    % to check how the total time is converginhg towards its minimum

    tOvrHd = 2*(tOut+(tEdge/2)+(v1Diff*tIn)/(3*v1Scan));
    tLine = d1Span/v1Scan+tOvrHd;
    %pdeltatOv=tOvrHd-tOvrHdOld;
    
    d1In = (tIn*(3*v1Scan-v1Diff))/3;
    
    myTextFormat = '\n      tLine-tEdge=%3.5f tEdge=%3.5f v1Out %3.4f|%3.4f| v1Diff %3.4f < %3.4f |%3.4f| <%3.4f d1In=%3.4f\n      ';
    fprintf(myTextFormat,tLine-tEdge,tEdge,v1Out,v1Out_opt,v1Diff_d1In_min,v1Diff,v1Diff_op_tLine,v1Diff_max,d1In);    
    thisIterChanges = '';

    % dtLine/dv1Scan v1Diff=cte
    dtLine_dv1Scan = (8*tIn*v1Scan^2-2*tIn*v1Diff^2+(-a1Out*tIn^2-4*d1Span)*v1Diff-2*a1Out*d1Span*tIn)/((4*v1Diff+2*a1Out*tIn)*v1Scan^2);        
    fprintf('dtLine_dv1Scan=%4.4f ',dtLine_dv1Scan)

    % dtLine/dv1Diff v1Scan=cte

    dtLine_dv1Diff= -(16*tIn*v1Scan^2+8*a1Out*tIn^2*v1Scan-4*tIn*v1Diff^2-4*a1Out*tIn^2*v1Diff-a1Out^2*tIn^3)/((8*v1Diff^2+8*a1Out*tIn*v1Diff+2*a1Out^2*tIn^2)*v1Scan);
    fprintf('dtLine_dv1Diff=%4.4f ',dtLine_dv1Diff)    
    

%% case ax1 turn-round is quicker than ax2
    if (tEdge > tSolverMin)
        
    % ax2 is SLOWER than ax1... try optimising d1Out
        %? ax2 is too slow, 
        % :decrease currently positive tEdge to minimum
        
        if (d1In > d1In_max)
            %% Directly reduce tIn, by reducing d2In, to decrease d1In            
            dumFactor = 0.8;
            
            d2In_max = (3*a2Max*d1In_max^2)/(2*(3*v1Scan-v1Diff)^2);
            dum = (1-dumFactor)*d2In + (dumFactor)*d2In_max;
            
            %dum = d2In*dumFactor;
            
            if abs(d2In - dum) > d2_ErTol
                d2In = dum;
                thisIterChanges = [thisIterChanges , sprintf('d2In:d1=%3.6f ',d2In)];
            end
            
            
            
        end
                % if VTol is used to make ax1 return faster, then
        if isempty(thisIterChanges) && (v1Diff > 0)
            %% Use LESS inner decceleration on Fly axis to decrease d1Out
            % slowing down ax1, 
            dumFactor = 2/3;
            pdeltaT=(tEdge/2)*dumFactor ;           
            tOutdum=tOut+pdeltaT; % eat up part of (tEdge/2) into tOut

            dum=-(tIn*(a1Out*tOutdum-2*v1Scan))/(2*(tOutdum+tIn));   

            if (dum > v1Diff_d1In_min)
                if (dum < 0)
                   v1Diff = 0;
                   thisIterChanges = [thisIterChanges , sprintf('v1Diff:e0=%3.4f ',v1Diff)];
                else
                    if (dum < v1Diff_max) 
                        v1Diff=dum; 
                        thisIterChanges = [thisIterChanges , sprintf('v1Diff:e1=%3.4f ',v1Diff)];
                    end
                end
            end
        end 
        if isempty(thisIterChanges) && (d1Out < d1OutHDM)
            %% d1Out is good, so try to get rid of the edge by reducing a1Out
            % this move usually helps j1Out but no gaurantee that this is
            % under j1HLM. However, at tEdge=0, if j1 is still above the
            % limit, it may mean that a1 and a2 shall both be scaled back
            dumFactor = 1 - t_erTol;
            pdeltaT=(tEdge/2)*dumFactor ;           
            tOutdum=tOut+pdeltaT; % eat up part of (tEdge/2) into tOut
            
            dum = (2*tIn*v1Scan+(-2*tOutdum-2*tIn)*v1Diff)/(tIn*tOutdum);
     
            a1Out_min= (2*v1Out*sqrt(tIn^2*v1Out^2-3*d1OutHDM*tIn*v1Diff)+2*tIn*v1Out^2-6*d1OutHDM*v1Diff)/(3*d1OutHDM*tIn);
            if a1Out_min < (2*v1Diff)/tIn
                a1Out_min = (2*v1Diff)/tIn;
            end
            
            if dum < a1Out_min
                dum = a1Out_min;
                fprintf('<')
            end
            
            if abs(a1Out - dum) > a1_ErTol
                a1Out = dum;
                thisIterChanges = [thisIterChanges , sprintf('a1Out:j=%3.4f ',a1Out)];
            end
            
        end        
    end      

    %% case ax2 turn-around is quicker
    % ax1 is not quick, need to increase currently negative tEdge to 0
    if (tEdge < -tSolverMin) 
           %% Consider chaniging velocities... based on sensitivity of tLine
        % don't try to optimise v1Diff for time!!!!  
        % see version #5 and #4

        % find max negative sensitivity
        dtLine_max = 0;
        if -dtLine_dv1Diff > dtLine_max
            dtLine_max = -dtLine_dv1Diff;
        end
        if (dtLine_dv1Scan > dtLine_max)
            dtLine_max = dtLine_dv1Scan ;
        end      
                
        if (dtLine_max == dtLine_dv1Scan) && (v1Out > v1ScanLLM)
            fprintf('!! dtLine_v1Scan don''t know !!')
            % thisPassLog = [thisPassLog , sprintf('v1Diff=%3.4f',v1Diff)];
        end
        
        if isempty(thisIterChanges) && (v1Out > v1ScanLLM)
            %% try increasing v1Diff for least j1Out
            % this may actually m,ake tEtge more negative but its best time
            % to do this!
            
            dumFactor = 1 - t_erTol;
            
            pdeltaT=(tEdge/2)*dumFactor ;           
            tOutdum=tOut+pdeltaT; % eat up part of (tEdge/2) into tOut
            dum=-(tIn*(a1Out*tOutdum-2*v1Scan))/(2*(tOutdum+tIn));            


            if dum < v1Diff_d1In_min
                dum = v1Diff_d1In_min;
            end

            if dum > v1Diff_max
                dum = v1Diff_max;
            end
            
            if abs(v1Diff - dum) > v1_ErTol
                v1Diff=dum;
                thisIterChanges = [thisIterChanges , sprintf('v1Diff:j=%3.4f ',v1Diff)];
            end
        end
        
        if isempty(thisIterChanges) && (dtLine_dv1Scan > 0 ) && (v1Scan > v1Diff + v1ScanLLM)
            %% Reduce v1Scan to gain on tLine
            
            dumFactor = 1-v1_ErTol;

            dum = v1Diff + v1Out_opt;

            dum = (1-dumFactor)*v1Scan + (dumFactor)*dum;
            
            if (dum > v1Scan_max)
                dum = v1Scan_max;
                fprintf('!to prevent d1In err!')
            end
            if (dum < v1Diff + v1ScanLLM)
                dum = v1Diff + v1ScanLLM ;
            end
            
            if abs(v1Scan - dum) > v1_ErTol
            
                v1Scan = dum;
            
                thisIterChanges = [thisIterChanges , sprintf('v1Scan:t=%3.4f ',v1Scan)];
            end
        end 
        
        if isempty(thisIterChanges) && (d1In > d1In_max)
            %% Directly reduce tIn, by reducing d2In, to decrease d1In            
            dumFactor = 1-d2_ErTol;
            
            d2In_max = (3*a2Max*d1In_max^2)/(2*(3*v1Scan-v1Diff)^2);
            dum = (1-dumFactor)*d2In + (dumFactor)*d2In_max;
            
            %dum = d2In*dumFactor;
            
            if abs(d2In - dum) > d2_ErTol
                d2In = dum;
                thisIterChanges = [thisIterChanges , sprintf('d2In:d1=%3.6f ',d2In)];
            end
            
            
            
        end        
        
        %% Reduce a2Max to get rid of tEdge
        if isempty(thisIterChanges) 
            
            % ax1 can't be made quicker
            % Decrease a2Max to increase currently negative tEdge to 0
            dumFactor = 1/8;
            dum=abs((6*d2Step)/((tOut+tIn)*(4*tOut+2*tIn+3*dumFactor*tEdge)));
            if dum < a2Max
                fprintf('<')
            else
                dum = a2Max * 0.9;
                fprintf('/')
            end

            % Inorder for the change to be retained, d2In has to change
            % accordingly. Otherwise tIn will be calculated based on
            % reduced a2Max and nullify its effect
            d2In = d2In *dum /a2Max;
            a2Max = dum;            
            
            thisIterChanges = [thisIterChanges, sprintf('a2Max:=%3.4f ',a2Max)];
        end
    end
 
    %% report & assert convergence
	fprintf('\n      ')
    fprintf(thisIterChanges)
    fprintf('\n')
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
    myTextFormat = '\n!! Solution NOT FOUND ... iter=%4.0f error=%4.3e\n';
    fprintf(myTextFormat,i,er);
    display(erLog(i-min(i-1,3):i-1,:));
    lineSubEnd=0;
    devError='   Solver Did Not Converge';
    d1Out=0;
    
    tOvrHd=0;
    return
end

if i < iterMax 
    assert(abs(v1Scan-v1Out-v1Diff)<1e-10,'ERROR');
    myTextFormat = '\n  =>%4.0f iters error=%4.3e\n';
    fprintf(myTextFormat,i,er);
    devError='Solved';
%% setup scan shaping via indentation


    d1IndRFun=[0.1,0.2, 0.3,  -0.2,-0.1,-0.2,-0.3];
    d1IndLFun= [-0.1,  -0.2,-0.3,  0.2,-0.1,-0.2,-0.3];
    
%     d1IndLFun=(cos([0:lineEnd/2-2]*pi/(lineEnd/4))-1)*d1Span;
%     d1IndRFun=d1IndLFun;
%     
%     d1IndLFun=(-cos([0:lineEnd/2-2]*pi/(lineEnd/4))+1)*d1Span;
%     d1IndRFun=-d1IndLFun;
    
%% report solution
   %fprintf('Found a solution:')
   %fprintf('A2_MAX=%0.5f, T_EDGE=%1.5f, T_IN=%2.5f, T_OUT=%3.5f, V1_DIFF=%4.5f, A1_OUT=%5.5f, v1Scan=%4.5f \n',a2Max,tEdge,tIn,tOut,v1Diff,a1Out,v1Scan)
end