function dz = Motions(t,z)
dz = zeros(5,1);

global  v0 AR  d S Cmq  c I lift drag tpx tpy CGW CGT CGP Cmo ksid Lb h K mpilot mwing tpdist cgdist phid sweepd Clwa Pil_act Pil_pos Glider

alpha=z(2)-z(4);

%Constants
g = 9.81;%N/kg or 
rho=1.22;%kg/m^3
                          

%Wing

switch Glider
    case 'Falcon'%Low inertia
        Cmo=0.05;
        I=102;% in kg.m² according to G.V , I=110;% according to Cook study. Inertia Pilot+ Glider about CGTotal
        AR=5.5;% Aspect ratio Falcon 5.5
        S=15.8;%m² Falcon 170 area
        c=1.9;% m Falcon 170
        mwing=22;% kg
        tpdist=8.5.*c./100;% en m (distance between CGW et Tether point)
        cgdist=0.06; % distance between tether point and the CGW .
        phid=14; % angle in degrees between the downtube and the axis perpendicular to the keel
        sweepd=31; % Sweep angle at the quarterchord in degrees
        Clwa=3.5;%spanwise average value of the wing section lift curve slope (slope of CL between 5 and 15 degrees of alpha)
        cbar=1.55; %distance between the apex of the downtubes and the control bar axis (optional)
        
        %Lift
        Clift=-16.6.*alpha.^3+11.48.*alpha.^2+1.3.*alpha+0.038;
        lift=0.5.*rho.*S.*Clift.*z(3).^2;

        %Drag
        Cdg=7.07.*alpha.^3-4.68.*alpha.^2+1.1.*alpha-0.0144;
        Dcdg=0.5.*Cdg.*rho.*S.*z(3).^2;     % Drag of the wing alone

        %Scx Pilot
        switch Pil_pos
        case 'Prone'
            Scx=    0.16;% S.Cx of the pilot position. If prone : Scx=0.16. If stood: Scx=0.66
        case 'Stand'
            Scx=    0.66;
        end

        Dpilot=0.5.*rho.*Scx.*z(3).^2;% Drag of the pilot alone
        drag=Dcdg+Dpilot;% Drag of the system (wing + pilot)
    
    case 'Falcon Hi' %High inertia
        Cmo=0.05;
        I=200;% in kg.m² according to G.V , I=110;% according to Cook study. Inertia Pilot+ Glider about CGTotal
        AR=5.5;% Aspect ratio Falcon 5.5
        S=15.8;%m² Falcon 170 area
        c=1.9;% m Falcon 170
        mwing=22;% kg
        tpdist=8.5.*c./100;% en m (distance between CGW et Tether point)
        cgdist=0.06; % distance between tether point and the CGW .
        phid=14; % angle in degrees between the downtube and the axis perpendicular to the keel
        sweepd=30; % Sweep angle at the quarterchord in degrees
        Clwa=3.5;%spanwise average value of the wing section lift curve slope (slope of CL between 5 and 15 degrees of alpha)
        cbar=1.55; %distance between the apex of the downtubes and the control bar axis (optional)
        
        %Lift
        Clift=-16.6.*alpha.^3+11.48.*alpha.^2+1.3.*alpha+0.038;
        lift=0.5.*rho.*S.*Clift.*z(3).^2;

        %Drag
        Cdg=7.07.*alpha.^3-4.68.*alpha.^2+1.1.*alpha-0.0144;
        Dcdg=0.5.*Cdg.*rho.*S.*z(3).^2;     % Drag of the wing alone

        %Scx Pilot
        switch Pil_pos
        case 'Prone'
            Scx=    0.16;% S.Cx of the pilot position. If prone : Scx=0.16. If stood: Scx=0.66
        case 'Stand'
            Scx=    0.66;
        end

        Dpilot=0.5.*rho.*Scx.*z(3).^2;% Drag of the pilot alone
        drag=Dcdg+Dpilot;% Drag of the system (wing + pilot)
    
    case 'T2C'
        Cmo=0.02;
        I=102;% in kg.m² according to G.V , I=110;% according to Cook study. Inertia Pilot+ Glider about CGTotal
        AR=7.3;% Aspect ratio Falcon 5.5
        S=13.4;%m² Falcon 170 area
        c=1.46;% m Falcon 170
        mwing=32;% kg
        tpdist=8.5.*c./100;% en m (distance between CGW et Tether point)
        cgdist=-0.12; % distance between tether point and the CGW .
        phid=14; % angle in degrees between the downtube and the axis perpendicular to the keel
        sweepd=21; % Sweep angle at the quarterchord in degrees
        Clwa=2.6;%spanwise average value of the wing section lift curve slope (slope of CL between 5 and 15 degrees of alpha)
        cbar=1.55; %distance between the apex of the downtubes and the control bar axis (optional)
        
        %Lift
        Clift=-2.74.*alpha.^2+4.4.*alpha+0.015;
        lift=0.5.*rho.*S.*Clift.*z(3).^2;

        %Drag
        Cdg=1.51.*alpha.^2-0.06.*alpha+0.03;
        Dcdg=0.5.*Cdg.*rho.*S.*z(3).^2;     % Drag of the wing alone

        %Scx Pilot
        switch Pil_pos
        case 'Prone'
            Scx=    0.16;% S.Cx of the pilot position. If prone : Scx=0.16. If stood: Scx=0.66
        case 'Stand'
            Scx=    0.66;
        end

        Dpilot=0.5.*rho.*Scx.*z(3).^2;% Drag of the pilot alone
        drag=Dcdg+Dpilot;% Drag of the system (wing + pilot)
        
end
     

AC=[0 0];%Center of the landmark ( aerodynamic center)
M=mpilot+mwing;
dyn=0.5.*rho.*z(3).^2.*S; % dynamic pressure
Pp=mpilot*g; %Force from mass
Pw=mwing*g;  %Force from mass  
phi=phid./360.*2.*pi; % conversion in radians
ksi=atan(Lb/h);%angle between cg pilot and the downtubes in radians
ksid=ksi./2./pi.*360;
sweep=sweepd./360.*2.*pi;

%Tether point (point d'accroche)
    tpx=tpdist.*cos(z(2));
    tpy=tpdist.*sin(z(2));

%The wing
    xCGW=tpx+cgdist.*cos(z(2));
    yCGW=tpy+cgdist.*sin(z(2));
    CGW=[xCGW yCGW]
%%
%Pilot center of gravity
    %Pilote maintains the angle ksi with the downtubes . Forward or Backward
 switch Pil_act
     case 'Trim';   %Real trim where pilot release totally the control bar
     xCGP=tpx;
     yCGP=tpy-h;
     
     case 'Active';
     xCGP=tpx+sin(z(2)+phi+ksi).*h;
     yCGP=tpy-cos(z(2)+phi+ksi).*h;
    
end
        
CGP=[xCGP yCGP]%Position CG pilot
%%
%Center of gravity total 
    xCGT=(mpilot.*xCGP+mwing.*xCGW)/M;
    yCGT=(mpilot.*yCGP+mwing.*yCGW)/M;
    CGT=[xCGT yCGT]% Position CG of the system (pilot+wing)

%The control bar
    xcbar=tpx+sin(z(2)+phi)*cbar
    ycbar=tpy-cos(z(2)+phi)*cbar;


%Tau=z(4);

Cx=-Dcdg;
Cy=lift;

d=norm(CGT);

%%
%Cmq variable (from Methods for Estimating Stability and Control Derivatives for Standard Subsonic Airplanes (1973) -Roskam p51  )

if AR<=6 
    K=0.7;
end

if AR>6&AR<10
    K=0.1*sin(2*pi*AR/8)+0.8;
end

if AR>=10 
    K=0.9;
end

Xw=xCGT;
Cmq=-K.*Clwa*cos(sweep)*((((1./24)*(AR.^3.*tan(sweep).^2)/(AR+6.*cos(sweep)))+1./8)+(AR*(2*(Xw/c)+0.5*(Xw/c))/(AR+2*cos(sweep))))
%%
%Damping
Mq=(Cmq.*z(5).*c.^2.*rho.*z(3).*S)./4;
Mq2=-0.5.*rho.*Cdg.*S.*(-2.*z(5).*d.^2.*z(3)+z(5).^2.*d.^3);%

XWT=xCGW-xCGT;
XP=xCGP-xCGT;
YP=yCGP-yCGT;
%Differential equations
dz(1)=z(3);
dz(2)=z(5);
dz(3)=-g.*sin(z(4))-(drag./M);
dz(4)=(1/(z(3)).*(-g*cos(z(4))+(lift./M)));
%dz(5)=(Cmo.*dyn.*c-Cy.*xCGT-Cx.*(-yCGT)-Pw.*XWT-Pp.*XP+Mq+Mq2)./I; %
%Without taking into account the drag of the pilot
%Primary version of dz(5)
%dz(5)=(-(-Cmo.*dyn.*c+(-Pw).*XWT+(-Pp).*XP+Cy.*xCGT+Cx.*(-yCGT)+(yCGP-yCGT).*Dpilot)+Mq+Mq2)./I;% drag of the pilot applied on CGP
%Simplification of the signs of dz(5)
dz(5)=(Cmo.*dyn.*c+Pw.*XWT+Pp.*XP-Cy.*xCGT-Cx.*(-yCGT)-(yCGP-yCGT).*Dpilot+Mq+Mq2)./I;% drag of the pilot applied on CGP

