clear all 
close all
 
v=20; %Airspeed in m/s
g = 9.81;%N/kg
rho=1.22;%kg/m^3

%Pilot Parameters
        mpilot=90;% kg (including harness and clothing = Hook-in weight)
        h=1.2; %in m (hang strap length)
        Lb=0.4; %0.4 if forward , -0.74 if backward :distance between the cgpilot and the axis of the control bar,if trim change 'Pil_act'
        Pil_act='Trim';% Choose Trim , or Active to make the pilot controlling the control bar or not.
        Pil_pos='Prone'; % Choose Stand or Prone
        
        Glider='Falcon';%Falcon or T2C

AC=[0 0];%Center of the landmark ( aerodynamic center)
%%
d=1; %First alpha
r=30; % Last alpha
t=1; %Difference between two values of alpha
alphad=zeros(1,r);
alphad(1)=d;
for i=2:t:r;
    alphad(i)=alphad(i-1)+t;
end

alpha=alphad./360.*2.*pi;
%%
switch Glider
    case 'Falcon'%Low inertia
        Cmo=0.05;
        I=102;% in kg.m² according to G.V , I=110;% according to Cook study. Inertia Pilot+ Glider about CGTotal
        AR=5.5;% Aspect ratio 
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
        lift=0.5.*rho.*S.*Clift.*v.^2;

        %Drag
        Cdg=7.07.*alpha.^3-4.68.*alpha.^2+1.1.*alpha-0.0144;
        Dcdg=0.5.*Cdg.*rho.*S.*v.^2;     % Drag of the wing alone

        %Scx Pilot
        switch Pil_pos
        case 'Prone'
            Scx=    0.16;% S.Cx of the pilot position. If prone : Scx=0.16. If stood: Scx=0.66
        case 'Stand'
            Scx=    0.66;
        end

        Dpilot=0.5.*rho.*Scx.*v.^2;% Drag of the pilot alone
        drag=Dcdg+Dpilot;% Drag of the system (wing + pilot)
      
    case 'T2C'
        Cmo=0.02;
        I=102;% in kg.m² according to G.V , I=110;% according to Cook study. Inertia Pilot+ Glider about CGTotal
        AR=7.3;% 
        S=13.4;%m² 
        c=1.46;% m 
        mwing=32;% kg
        tpdist=8.5.*c./100;% en m (distance between CGW et Tether point)
        cgdist=-0.12; % distance between tether point and the CGW .
        phid=14; % angle in degrees between the downtube and the axis perpendicular to the keel
        sweepd=21; % Sweep angle at the quarterchord in degrees
        Clwa=2.6;%spanwise average value of the wing section lift curve slope (slope of CL between 5 and 15 degrees of alpha)
        cbar=1.55; %distance between the apex of the downtubes and the control bar axis (optional)
        
        %Lift
        Clift=-2.74.*alpha.^2+4.4.*alpha+0.015;
        lift=0.5.*rho.*S.*Clift.*v.^2;

        %Drag
        Cdg=1.51.*alpha.^2-0.06.*alpha+0.03;
        Dcdg=0.5.*Cdg.*rho.*S.*v.^2;     % Drag of the wing alone

        %Scx Pilot
        switch Pil_pos
        case 'Prone'
            Scx=    0.16;% S.Cx of the pilot position. If prone : Scx=0.16. If stood: Scx=0.66
        case 'Stand'
            Scx=    0.66;
        end

        Dpilot=0.5.*rho.*Scx.*v.^2;% Drag of the pilot alone
        drag=Dcdg+Dpilot;% Drag of the system (wing + pilot)
        
end
%%
dyn=0.5.*rho.*v.^2.*S;
Cdrag=drag./dyn;

Cx=-Dcdg;
Cy=lift;

%Tau=gamma;
%Cx=-drag.*cos(Tau)-lift.*sin(Tau);
%Cy=lift.*cos(Tau)-drag.*sin(Tau);
%if drag and lift expressed in another basis (wing)

gamma=-atan(Cdrag./Clift);
theta=gamma+alpha;
phi=phid./360.*2.*pi; % conversion in radians
ksi=atan(Lb/h);%angle between cg pilot and the vertical in radians
ksid=ksi./2./pi.*360;
gammad=gamma.*360/(2*pi);
thetad=theta*360/(2*pi);

key=sin(theta+phi+ksi)*h;
M=mpilot+mwing;
%%
%Tether point (point d'accroche)
tpx=tpdist.*cos(theta);
tpy=tpdist.*sin(theta);

%Pilot center of gravity
    %Pilote maintains the angle ksi with the downtubes . Forward or Backward
 switch Pil_act
     case 'Trim';   %Real trim where pilot release totally the control bar
     xCGP=tpx;
     yCGP=tpy-h;
     
     case 'Active';
     xCGP=tpx+sin(theta+phi+ksi).*h;
     yCGP=tpy-cos(theta+phi+ksi).*h;
    
end
        
%Center of gravity of the wing
xCGW=tpx+cgdist.*cos(theta);
yCGW=tpy+cgdist.*sin(theta);

%Position of the control bar ( optional ) 
xcbar=tpx+sin(theta+phi)*cbar;
ycbar=tpy-cos(theta+phi)*cbar;

%Center of gravity total 
xCGT=(mpilot.*xCGP+mwing.*xCGW)/M;
yCGT=(mpilot.*yCGP+mwing.*yCGW)/M;
CGT=[xCGT yCGT];% Position CG of the system (pilot+wing)
%%
Pp=mpilot*g;
Pw=mwing*g;

W=xCGW;
T=-yCGT;
XWT=xCGW-xCGT;
XP=xCGP-xCGT;
YP=yCGP-yCGT;

P=xcbar-xCGP;
Q=ycbar-yCGP;
R=sqrt(P.^2+Q.^2);

Smom=-(-Cmo.*dyn.*c+(-Pw).*XWT+(-Pp).*XP+Cy.*xCGT+Cx.*T+(yCGP-yCGT).*Dpilot);

SM=norm(CGT);

A=[alphad' gammad' Smom' Clift' Cdrag'];

%Approximation of inertia ( optional )
alfa=20
WT=[xCGT(alfa)-xCGW(alfa),yCGT(alfa)-yCGW(alfa)]
PT=[xCGT(alfa)-xCGP(alfa),yCGT(alfa)-yCGP(alfa)]
b=9.3 % span in m
mvit=rho*pi*(c/2)^2*b
q1=norm(WT)
q2=norm(PT)
In=mwing*q1^2+16.9+mpilot*q2^2+mvit*q1^2 % Approximation of inertia

figure;
plot(A(:,1),A(:,3),'b');
legend('Sum of moments curve')
xlabel('Angle of attack (deg)');
ylabel('Sum of moments about CGT (N.m)');
%hold on
%plot(A(:,1),A(:,2),'r');
hold on
plot([0 r],[0 0],'g');

