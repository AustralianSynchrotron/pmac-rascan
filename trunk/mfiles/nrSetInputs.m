function [] = nrSetInputs()


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

%% inputs

%System specification

pE1Res=0.0000001;
pE2Res=0.0000001;

% Geometry
ax1OffUsr=0;
ax1LLMUsr=-10000;
ax1HLMUsr=10000;

ax2OffUsr=0;
ax2LLMUsr=-10000;
ax2HLMUsr=10000;

% Scan definition
d1Span=2;
d1Centre=0;
v1Scan=1;


d2Start=10;
d2Step=1;%mm


% Stage spec
a1HLM=5; %mm/s^2
a2HLM=1;%mm/s^2

j1HLM=1000; %mm/s^3
j2HLM=100; %mm/s^3

% Rascan limitations
tInLLM=0.005;
tMidLLM=0.002;
tHLM=4;

d1IndLReq=0;
d1IndRReq=0;

lineEnd=10;
