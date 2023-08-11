%% SXR FAM coord system test

m4X0Usr=0;
m4MRES=0.001;
m4PoffUsr=0;


m5X0Usr=0;
m5MRES=0.001;
m5PoffUsr=0;


m4VelCRaw=0;
m4PrbvUsr=0;


m5VelCRaw=0;
m5PrbvUsr=0;

UPosCUsr=10
VPosCUsr=2
    
%% Forward
m4PinvUsr = m4X0Usr + UPosCUsr
	csZR1 = m4PinvUsr;
	SinZR1 = sin(csZR1);
	CosZR1 = cos(csZR1);
	
	m5PinvUsr = m5X0Usr + VPosCUsr - 18/360*csZR1


	m4PosCRaw = (m4PinvUsr - m4PoffUsr)/m4MRES
	m5PosCRaw = (m5PinvUsr - m5PoffUsr)/m5MRES
	
	
	m4VelCRaw = UVelCUsr/m4MRES;
	m5VelCRaw = VVelCUsr/m5MRES - 18/360*UVelCUsr ;   
    
%% 
m4PosCUsr=(m4MRES*m4PosCRaw+m4PoffUsr)
m5PosCUsr=(m5MRES*m5PosCRaw+m5PoffUsr)
    
    
    %% Inverse 
csZR1 = m4PosCUsr;
	SinZR1 = sin(csZR1);
	CosZR1 = cos(csZR1);
	%! check for div by 0
	
	UPosCUsr = m4PosCUsr - m4X0Usr
	UVelCUsr = m4VelCRaw*m4MRES;
	VPosCUsr = m5PosCUsr - m5X0Usr + 18/360*csZR1
	VVelCUsr = m5VelCRaw*m5MRES + 18/360*UVelCUsr;