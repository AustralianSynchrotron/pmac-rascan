function [] = nrRascan_Prog()

%% interface definition

%Scan instance inputs
global d1Centre d2Start lineN lineEnd     
% Status & info variables
global devError tLine tOvrHd 
%CS parameters. Not in solver
global ax1HLMUsr ax1LLMUsr ax2HLMUsr ax2LLMUsr ax1OffUsr ax2OffUsr pE1Res pE2Res vCycles
% Trajectory solver essential inputs
global tInLLM d1InHLM d1OutHDM d1Span d2TolRatio d2Step v1TolRatio v1ScanHLM v1ScanLLM a1HLM a2HLM j1HLM j2HLM 
% Trajectory solver essential outputs
global tIn tOut tEdge d1Out v1Scan v1Diff a1Out a2Max

% variables set by Rascan motion program
global tMid tHLM lineSubEnd 

% Rascan Indent
global d1IndL d1IndR d1IndHLM d1IndLReq d1IndRReq

global SampleTime ax1tsMotion ax2tsMotion  d1IndRFun d1IndLFun m_IndentationOn %simulation variables

% simulated jerk values
global j1In j1Out j2In j2Out j2Edge 

%% presets
tHLM=4;
tAHLM=4.095;
tMidLLM=0.05;
tALLM=0.000002;


%% ----- Rascan Program simulation
  
    %digitize and eliminate round-offs:

    v1Diff=floor(v1Diff/pE1Res/36/SampleTime)*pE1Res*36*SampleTime;
    v1Scan=floor(v1Scan/pE1Res/36/SampleTime)*pE1Res*36*SampleTime;

    tEdge=floor(tEdge/SampleTime+0.5)*SampleTime;
    tIn=floor(tIn/SampleTime+0.5)*SampleTime;
    tOut=floor(tOut/SampleTime+0.5)*SampleTime;

%% PMAC BLOCK ;; {{ ------------------- Compiling: PVT calculations
	lineDir=1-2*(mod(lineN,2));
    v1Out=v1Scan-v1Diff;
    palpha=(a1Out*tIn+2*v1Diff);
    d1In=((3*v1Out+2*v1Diff)*tIn)/3;
    d1Out=(4*(palpha-v1Diff)*v1Out*v1Out*tIn)/(3*palpha*palpha);
    %(o585) 
    a2Out=(a2Max*tEdge)/(2*tOut+tEdge);
    %(d2InEq)
    d2In=(a2Max*tIn*tIn)/6;
    %(v2InEq)
    v2In=(a2Max*tIn)/2;
    d2Out=(tOut*(a2Out*tOut+2*a2Max*tOut+3*a2Max*tIn))/6;
    v2Out=v2In+tOut*(a2Max+a2Out)/2;
    

    
% calculate Edge move    
% if ESkip is almost zero ... keep it there while searching for a min
    d2Edge=(d2Step-2*d2In-2*d2Out);
    if (d2Edge<pE2Res)
       tEdge=0;
    end
	if (tEdge<0.002)
       d2Edge=0;
       tEdge=0;
       d2Out=d2Step/2-d2In;
    end
    
% Ax1Asym begins here ------------------------------------

    tEdge1=0;    d1Edge1=0;    v1Edge1=0;    tEdge2=0;    d1Edge2=0;    v1Edge2=0;    
    d2Edge1=0;    v2Edge1=0;    d2Edge2=0;    v2Edge2=0;        v3=v1Out;
    
    if tEdge > 0

        a1Max=a1HLM;
        a1In=v1Diff/tIn*2;

        v3=v1Scan*v1TolRatio;
        a3=(v1Out-v3)/tIn*3;

        tEdge1=(a3*tOut^2+a3*tEdge*tOut-4*v3*tOut-2*v3*tEdge+6*d1Out)/(a1Max*tOut+a3*tOut+2*a1Max*tEdge-2*v3);
        
        if false && (tEdge1 > tALLM) && (tEdge1 < tEdge + tALLM)
        
            %ASymmetric solution
            a2=-(2*a3*a1Max*tOut^2+a3^2*tOut^2+3*a3*a1Max*tEdge*tOut-6*v3*a1Max*tOut-4*a3*v3*tOut-6*v3*a1Max*tEdge+6*a1Max*d1Out+4*v3^2)/(a1Max*tOut^2+3*a1Max*tEdge*tOut+2*v3*tOut+2*a1Max*tEdge^2-6*d1Out)

            d1Edge1=(a1Max*(a3*tOut^2+a3*tEdge*tOut-4*v3*tOut-2*v3*tEdge+6*d1Out)^2)/(3*(a1Max*tOut+a3*tOut+2*a1Max*tEdge-2*v3)^2)
            v1Edge1=(a1Max*(a3*tOut^2+a3*tEdge*tOut-4*v3*tOut-2*v3*tEdge+6*d1Out))/(2*(a1Max*tOut+a3*tOut+2*a1Max*tEdge-2*v3))
            d1Edge2=-((a3*tOut^2-a1Max*tEdge*tOut-4*v3*tOut-2*a1Max*tEdge^2+6*d1Out)*(3*a3*a1Max^2*tOut^4+2*a3^2*a1Max*tOut^4+a3^3*tOut^4+10*a3*a1Max^2*tEdge*tOut^3+2*a3^2*a1Max*tEdge*tOut^3-12*v3*a1Max^2* ...
            tOut^3-8*a3*v3*a1Max*tOut^3-8*a3^2*v3*tOut^3+8*a3*a1Max^2*tEdge^2*tOut^2-2*a3^2*a1Max*tEdge^2*tOut^2-36*v3*a1Max^2*tEdge*tOut^2-8*a3*v3*a1Max*tEdge*tOut^2+18*a1Max^2*d1Out*tOut^2+6*a3^2*d1Out*tOut^2+20*a3*v3^2*tOut^2- ...
            24*v3*a1Max^2*tEdge^2*tOut+8*a3*v3*a1Max*tEdge^2*tOut+48*a1Max^2*d1Out*tEdge*tOut+8*v3^2*a1Max*tEdge*tOut+48*v3*a1Max*d1Out*tOut-24*a3*v3*d1Out*tOut-16*v3^3*tOut+24*a1Max^2*d1Out*tEdge^2-8*v3^2*a1Max*tEdge^2- ...
            72*a1Max*d1Out^2+24*v3^2*d1Out))/(6*(a1Max*tOut+a3*tOut+2*a1Max*tEdge-2*v3)^2*(a1Max*tOut^2+3*a1Max*tEdge*tOut+2*v3*tOut+2*a1Max*tEdge^2-6*d1Out))
            v1Edge2=(a3*a1Max*tOut^3+a3^2*tOut^3-4*v3*a1Max*tOut^2-6*a3*v3*tOut^2-2*a3*a1Max*tEdge^2*tOut+6*a1Max*d1Out*tOut+6*a3*d1Out*tOut+8*v3^2*tOut+4*v3*a1Max*tEdge^2-12*v3*d1Out)/(2*(a1Max*tOut^2+3*a1Max*tEdge*tOut+2*v3*tOut+2*a1Max*tEdge^2-6*d1Out))

            %Symmetric solution
            
            
            
            
            d2Edge1=(v2Out + 1/2*a2Out*tEdge1)*tEdge1
            v2Edge1=v2Out+a2Out*tEdge1;

            d2Edge2=d2Edge-d2Edge1;
            v2Edge2=v2Out;
        else
            tEdge1=0; d1Edge1=0; v1Edge1=0; d1Edge2=0; v1Edge2=0;
            v3=v1Out; 
            
            d2Edge1=0; v2Edge1=0; d2Edge2=d2Edge; v2Edge2=v2Out;
        end
    end

    d1Out2=d1Out-d1Edge1-d1Edge2;    
    v1Out2=v3;
    
% Ax1Asym ends here ------------------------------------
    
    d1IndHLM=(-(tMidLLM+0.001)*v1Scan+d1Span-2*d1In);
    d1IndL=0;
    d1IndR=0;
    tOvrHd = 2*(tOut+(tEdge/2)+(v1Diff*tIn)/(3*v1Scan));
    


% PMAC BLOCK ;; }} -------------------- Compiling: PVT calculations
    
    
%% State machine control
    devError=[];
    fprintf('validating...')
	   
%% PMAC BLOCK ;; {{ -------------------- Validation
    if (tEdge<0)
        devError='(tEdge<0)';
    end
    if (d2In < 0)
        devError='(d2In < 0)';
    end

    if (tIn < tInLLM)
		devError='erERROR8';
    end
    if (d2Edge < 0)
        devError='d2Edge < 0';
    end
    
    a1In=v1Diff/tIn*2;
	if (a1In > a1HLM*1.01) 
		devError='era1In';
    end
%----------------
	tMid=(d1Span-2*d1In)/v1Scan;
	lineSubEnd=0;
	if (tMid > tHLM)
		while (tMid > tHLM)
            lineSubEnd=lineSubEnd+1;
			tMid=tMid - tHLM;
        end
		%now if tMid is too small, add one tHLM to tMid and deduct one from subsegments
		if(tMid < tAHLM-tHLM)
			tMid=tMid+tHLM;
			lineSubEnd=lineSubEnd-1;
        end
    end
	%tMid should be less than PVT acceptable limit 
	if (tMid > tAHLM)
		devError='tMid > tAHLM';
    end
	if (tMid < 0.002)
		devError='tMid < 0.002';
    end
%------------------------
	if (tIn > tAHLM)
		devError='(tIn > tAHLM)';
    end
	if (tOut > tAHLM)
		devError='tOut > tAHLM';
    end
	if (tEdge > tAHLM)
		devError='tEdge > tAHLM';
    end
	if (d1Span<2*d1In)
		devError='d1Span<2*d1In';
    end
	%Check Boundaries and SOFT limits. If Inconsistent then
	pX0= d1Out + (d1Centre+d1Span/2)- ax1OffUsr;
	if (pX0> ax1HLMUsr)
		devError='erRange1';
    end
	pX0= -d1Out + (d1Centre-d1Span/2)- ax1OffUsr;
	if (pX0< ax1LLMUsr)
		devError='erRange1';
	end
    pY0=d2Step*(lineN-1)+d2Start-ax2OffUsr;
	pY1=d2Step*(lineEnd)+d2Start+d2In+d2Out-d2Edge/2/2-ax2OffUsr;
	if (d2Step < 0)
		if ( pY0 > ax2HLMUsr)
			devError='erRange2';
        end
		if ( pY1 < ax2LLMUsr)
			devError='erRange2';
        end
	else
		if ( pY1 > ax2HLMUsr)
			devError='erRange2';
        end
		if ( pY0 < ax2LLMUsr)
			devError='erRange2';
        end
    end
    
	% PMAC BLOCK ;; }} -------------------- Validation
 %% Matlab output
    %d1IndentTotal=0;
    tLine = 2 * tOut + 2 * tIn + tEdge + tMid + lineSubEnd * tHLM;
        
    myTextFormat = 'v1Diff/v1Scan=%4.4f d2In/d2Step=%4.4f \ntOut=%4.4fs (tEdge/2)=%4.4fs tOvrHd=%4.4fs tLine=%4.4fs d1Out=%6.5fmm d1In=%6.5fmm tIn=%4.4fs \n';
    fprintf(myTextFormat,v1Diff/v1Scan,d2In/d2Step,tOut,(tEdge/2),tOvrHd,tLine,d1Out,d1In,tIn);
    
    testCriteria='(d2In/d2Step>d2TolRatio*1.1)';
    if eval(testCriteria)
        display('*** Not acceptable: ',testCriteria')
    end
    
    testCriteria='(v1Diff/v1Scan > v1TolRatio*1.1)';
    if eval(testCriteria)
        display('**** Not acceptable: ',testCriteria')
    end    
    
    
    
    %% Error cheching ... this section is implemented in solver, not in PMAC
   
    if ~isempty(devError)
        fprintf('!!!!! - devError : %s',devError);
        return
    end
    
% Verify if jerks are within limits =========================
    j1In=(a1In-0)/tIn;
    j1Out=(a1Out-a1In)/tOut;
    
    j2In=((v2In/tIn*2)-0)/tIn;
    j2Out=(a2Out-j2In*tIn)/tOut;
    j2Edge=(0-a2Out)/(tEdge/2+tALLM);
    
    % evaluate jerk values
    jErrors = '';
    fprintf(check_print(j1In,j1HLM, ''));
    fprintf(check_print(j1Out,j1HLM, ''));  
    fprintf(check_print(j2In,j2HLM, ''));   
    fprintf(check_print(j2Out,j2HLM, ''));  
    fprintf(check_print(j2Edge,j2HLM, '')); 
    fprintf('\n')
    
% ----------------------------------------------------------           
    
    fprintf('simulating motion')

    %% PMAC BLOCK ;; {{ -------------------- Setting
    t=-tOut;
    pX0=(1-lineDir)/2*d1Span -lineDir*d1Out + (d1Centre-d1Span/2) - ax1OffUsr;
    ax1PointA=[pX0,0,t];
    ax2PointA=[pY0,0,t];

    ax1tsMotion=timeseries('Augted');
    ax2tsMotion=timeseries('Augted');

    % PMAC BLOCK ;; }} -------------------- Setting
	

	%% PMAC BLOCK ;; {{ -------------------- Running: initial ramp up
    %pre-move to ramp up ax2, without moving ax1
    fprintf('.')

    %ISkip accel for ax2 only
    pdeltat=tIn;
    ax1PointBInc=[0,0,pdeltat];
    ax2PointBInc=[d2In,v2In,pdeltat];

    [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res,vCycles); 
    [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res,vCycles);

    %OSkip accel. for ax2 only
    pdeltat=tOut;
    ax1PointBInc=[0,0,pdeltat];
    ax2PointBInc=[d2Out,v2Out,pdeltat];

    [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res,vCycles); 
    [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res,vCycles);
    
    %ESkip 
    pdeltat=tEdge;
    ax1PointBInc=[0,0,pdeltat];
    ax2PointBInc=[d2Edge,v2Out,pdeltat];

    [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res,vCycles); 
    [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res,vCycles);
    
	% PMAC BLOCK ;; }} -------------------- Running: initial ramp up
	
	%% PMAC BLOCK ;; {{ -------------------- Running: main loop
    
    while (lineN<lineEnd)
        fprintf('.')
        %OSkip accel.
        pdeltat=tOut;
        ax1PointBInc=[lineDir*d1Out2,lineDir*v1Out2,pdeltat];
        ax2PointBInc=[d2Out,v2In,pdeltat];

        [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res,vCycles); 
        [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res,vCycles);

        %ISKip accel.
        pdeltat=tIn;
        ax1PointBInc=[lineDir*d1In,lineDir*v1Scan,pdeltat];
        ax2PointBInc=[d2In,0,pdeltat];

        [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res,vCycles); 
        [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res,vCycles);



		%% Scan forward
        pdeltat=tMid+(lineSubEnd)*tHLM;
       
        ax1PointBInc=[lineDir*pdeltat*v1Scan,lineDir*v1Scan,pdeltat];
        ax2PointBInc=[0,0,pdeltat];

        [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res,vCycles); 
        [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res,vCycles);

        %ISkip deccel.
        pdeltat=tIn;
        ax1PointBInc=[lineDir*d1In,lineDir*v1Out,pdeltat];
        ax2PointBInc=[d2In,v2In,pdeltat];

        [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res,vCycles); 
        [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res,vCycles);

%% Indentation functionality

if m_IndentationOn
%% Indent control high level operation, via epics

        if (lineDir<0)
            if ((lineN+1)/2<length(d1IndLFun))
				d1IndLReq=d1IndLFun((lineN+1)/2);
            end
        else
            if ((lineN/2+1)<length(d1IndRFun))
				d1IndRReq=d1IndRFun(lineN/2+1);
            end
        end
       
%% PMAC BLOCK ;; {{ -------------------- Running: Indentation control
        if (lineDir<0)
            if (d1IndLReq+d1IndR < d1IndHLM)
                d1IndL=d1IndLReq;
            else
                % signal the indenter to compensate into next indent
                d1IndL=d1IndHLM-d1IndR;
            end
        else
            if (d1IndL+d1IndRReq < d1IndHLM)
                d1IndR=d1IndRReq;
            else
                % signal the indenter to compensate into next indent
                d1IndR=d1IndHLM-d1IndL;
            end
                
        end
            
        d1IndTotal= d1IndL+d1IndR;               
        
        
% tMid adaptation for indentation
        tMid=(d1Span-d1IndTotal-2*d1In)/v1Scan;
        
        testDum='tMid >= tMidLLM';
        if ~eval(testDum) 
            display(strcat('ERROR:',testDum))
            tMid
        end
        
        lineSubEnd=0;
        if (tMid > tHLM)
            while (tMid > tHLM)
                lineSubEnd=lineSubEnd+1;
                tMid=tMid - tHLM;
            end
            %now if tMid is too small, add one tHLM to tMid and deduct one from subsegments
            if(tMid < tMidLLM)
                tMid=tMid+tHLM;
                lineSubEnd=lineSubEnd-1;
            end
        end
        %tMid should be less than PVT acceptable limit 

        if (tMid > tAHLM)
            devError='erTrjBuilder';
        end

        if (tMid < 0.002)
            devError='erd1In';
        end
% PMAC BLOCK ;; }} -------------------- Running: Indentation control  
end
        % next line
        lineN=lineN+1;
        lineDir=1-2*(mod(lineN,2));
%OSkip deccel.
        pdeltat=tOut;
        ax1PointBInc=[-lineDir*d1Out,0,pdeltat];
        ax2PointBInc=[d2Out,v2Out,pdeltat];

        [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res,vCycles); 
        [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res,vCycles);
%ESkip 
        pdeltat=tEdge1;
        ax1PointBInc=[lineDir*d1Edge1,lineDir*v1Edge1,pdeltat];
        ax2PointBInc=[d2Edge1,v2Edge1,pdeltat];

        [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res,vCycles); 
        [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res,vCycles);
        
        pdeltat=tEdge-tEdge1;
        ax1PointBInc=[lineDir*d1Edge2,lineDir*v1Edge2,pdeltat];
        ax2PointBInc=[d2Edge2,v2Edge2,pdeltat];

        [ax1PointA, ax1tsMotion]=nrIncPVT(ax1tsMotion,ax1PointA,ax1PointBInc,SampleTime,pE1Res,vCycles); 
        [ax2PointA, ax2tsMotion]=nrIncPVT(ax2tsMotion,ax2PointA,ax2PointBInc,SampleTime,pE2Res,vCycles);
        
        
        
    end
    % PMAC BLOCK ;; }} -------------------- Running: main loop
