function [] = nrRascan_Prog()

%% interface definition

%Scan instance inputs
global d1Centre d2Start lineN lineEnd     
% Status & info variables
global devError tLine tOvrHd 
%CS parameters. Not in solver
global ax1HLMUsr ax1LLMUsr ax2HLMUsr ax2LLMUsr ax1OffUsr ax2OffUsr pE1Res pE2Res
% Trajectory solver essential inputs
global tInLLM d1InHLM d1OutHDM d1Span d2TolRatio d2Step v1TolRatio v1ScanHLM v1ScanLLM a1HLM a2HLM j1HLM j2HLM 
% Trajectory solver essential outputs
global tIn tOut tEdge d1Out v1Scan v1Diff a1Out a2Max

% variables set by Rascan motion program
global tMid tHLM lineSubEnd 

% Rascan Indent
global d1IndL d1IndR d1IndHLM d1IndLReq d1IndRReq

global SampleTime ax1tsMotion ax2tsMotion  d1IndRFun d1IndLFun m_IndentationOn %simulation variables

%% presets
tHLM=4;
tAHLM=4.095;
tMidLLM=0.05;
tALLM=0.000002;


%% ----- Rascan Program simulation


%% PMAC BLOCK ;; {{ ------------------- Compiling: PVT calculations
 


% PMAC BLOCK ;; }} -------------------- Compiling: PVT calculations
    
    
%% State machine control
    devError=[];
    fprintf('validating...')
	   
%% PMAC BLOCK ;; {{ -------------------- Validation

    
	% PMAC BLOCK ;; }} -------------------- Validation
 %% Matlab output
    %d1IndentTotal=0;
    
    tIn = 0.1;
    tEdge = 0.1;
    tMid = 0.1;
    tOut = 0.1;
    lineSubEnd = 0;
    
    tLine = 2 * tOut + 2 * tIn + tEdge + tMid + lineSubEnd * tHLM;
    
    %% Error cheching ... this section is implemented in solver, not in PMAC
   
    if ~isempty(devError)
        fprintf('!!!!! - devError : %s',devError);
        return
    end
    
% Verify if jerks are within limits =========================

% ----------------------------------------------------------           
    
    fprintf('simulating motion')

    pdeltat=0.1;     
    
    d1Old=0; 
    d2Old=0;
    ax1PointA=[d1Old,0,0];
    ax2PointA=[d2Old,0,0];

    ax1tsMotion=timeseries('Augted');
    ax2tsMotion=timeseries('Augted');
    
    d1=-1;
    d2=-5;
    ax1PointBInc=[d1-d1Old,0,pdeltat];
    ax2PointBInc=[d2-d2Old,0,pdeltat];

    [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res); 
    [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res);

    
 	d1Old=d1;
    d2Old=d2;
    
    lineEnd = 4;
    lineN=-lineEnd+1;   
    pdeltat=0.1;    
    
    while (lineN<lineEnd)
        
        fprintf('.')

        d2=lineN;
        
        d1_target=sampleTraj(d2,4,1.5,1);

        %%Look ahead: 
        % HOW will the next move be? find d1 and v1 and pdeltat 
        % for given d1Old v1Old and a1Old values      
        % and LLM constraints
        % so that (?) d1_error is minimised for this move
        
        % start by back propagating delta x on constant jerk (PVT, assuming
        % that none of the kinematic limits will be hit:
        
        
        d1=d1_target; %  - sign(d1_target-d1Old)*0.1
        v1=(d1-d1Old+d1Out*0.5)/pdeltat;        
                    
        ax1PointBInc=[d1-d1Old,v1,pdeltat];

        v2=(d2-d2Old)/pdeltat;
        ax2PointBInc=[d2-d2Old,v2,pdeltat];

        [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res); 
        [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res);
        
        d1Old=d1;
        d2Old=d2;
        
        lineN=lineN+1;
        
    end
    % PMAC BLOCK ;; }} -------------------- Running: main loop
