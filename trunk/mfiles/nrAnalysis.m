%% interface definition

global d1Centre d2Start lineEnd tInLLM a1HLM v1Scan d2Step d1Span j1HLM j2HLM %Rascan gsu
global tMid tOut lineN lineSubEnd devError tEdge tIn v1Diff a2Max d1Out tOvrHd a1Out % Rascan gs
global ax1HLMUsr ax1LLMUsr ax2HLMUsr ax2LLMUsr ax1OffUsr ax2OffUsr pE1Res pE2Res %CS
global d2TolRatio v1TolRatio a2HLM d1InHLM %Rascan Solver
global d1IndL d1IndR d1IndHLM d1IndLReq d1IndRReq% Rascan Indent
global SampleTime ax1tsMotion ax2tsMotion tHLM d1IndRFun d1IndLFun m_IndentationOn %simulation


%%
pdelta=d2Step/a2Max;
l=50;
tOvrHd=zeros(l,l);
tEdge=zeros(l,l);
d1Out=zeros(l,l);
v1Diff=linspace(0,0.99,l).*v1Scan;
tIn=linspace(tInLLM,(d1Span/2*0.95)/v1Scan,l)';
for i = 1:l
    tOvrHd(:,i)=(tIn.^2+24*pdelta).^0.5./4+(v1Diff(i).*tIn)/(3*v1Scan)-(3.*tIn)/4;
    tEdge(:,i)=(sqrt(tIn.^2+24*pdelta)-3*tIn)/4-(2*(v1Scan-v1Diff(i))*tIn)./(a1HLM*tIn+2*v1Diff(i))*2;
    d1Out(:,i)=(4*(v1Scan-v1Diff(i))^2*tIn.*(a1HLM*tIn+v1Diff(i)))./(3*(a1HLM*tIn+2*v1Diff(i)).^2);
end

tOvrHd=tOvrHd';
tEdge=tEdge';
d1Out=d1Out';
myMeshResultNoLog(tIn,v1Diff,tOvrHd,'tIn','v1Diff','tOvrHd');
myMeshResultNoLog(tIn,v1Diff,tEdge/2,'tIn','v1Diff','tEdge/2');
myMeshResultNoLog(tIn,v1Diff,d1Out,'tIn','v1Diff','d1Out');


myMeshResultNoLog(tIn,v1Diff,tOvrHd./(tEdge>0),'tIn','v1Diff','tOvrHd tEdge>0');
myMeshResultNoLog(tIn,v1Diff,d1Out./(tEdge>0),'tIn','v1Diff','d1Out tEdge>0');

myMeshResultNoLog(tIn,v1Diff,(tOvrHd+d1Out)./(tEdge>0),'tIn','v1Diff','(tOvrHd+d1Out) tEdge>0');

