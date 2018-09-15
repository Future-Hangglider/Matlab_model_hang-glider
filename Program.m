clear all
close all

global v0 AR  d S Cmq  c I lift drag tpx tpy CGW CGT CGP Cmo ksid Lb h K mpilot mwing tpdist cgdist phid sweepd Clwa Pil_act Pil_pos Glider
%declare global var allows us to see them in Command Window
%%
%Definition of the domain allowed (for the warning message and the limits of display of alpha plots): 
        alphamin=-5;
        alphamax=30;
%%
%Initial conditions (Step 1)
        alph0=20;%angle of attack initial en degrees (alph=a+f)
        u0 =0 ; %initial position (0) 
        a0 =-10; %flight path angle in degrees Gamma
        b0=0 ;%Pitch rate attitude in degrees/sec� Theta dot
        v0=9;%initial airspeed in m/s
        
                %Transformation in radians
                alphr0=alph0.*pi./180;
                ar0=(a0*pi)/180;%flight path angle in radians 
                fr0=alphr0+ar0;% pitch attitude in radians/sec Theta
                br0=b0.*pi./180;%Pith rate attitude in radians/sec�
        
        timespan=0.1;
        T0 =0;
        T1 =8;% in seconds
%%
%Parameters 
        %Pilot
        mpilot=90;% kg (including harness and clothing = Hook-in weight)
        h=1.2; %in m (hang strap length)
        Lb=0.4; %0.4 if forward , -0.74 if backward :distance between the cgpilot and the axis of the control bar,if trim change 'Pil_act'
        Pil_act='Active';% Choose Trim , or Active to make the pilot controlling the control bar or not.
        Pil_pos='Prone'; % Choose Stand or Prone
        
        Glider='Falcon';%Falcon or T2C
        
[t_1,z_1] = ode45('Motions',T0:timespan:T1,[u0 fr0 v0 ar0 br0]); % Runge-Kutta 4th and 5th order
export='N' ; % Y to activate the export, N to unable the export to spreadsheet.
filename=['Forward_fly','.xls'];
Step2='Y'; %(Activate ('Y') or inactivate step2 ('N'). No export if Step 2 activated in Parallel
Conf_step2='Next';% Step 2 next to Step 1 ('Next')  or in the same time  to compare 2 configurations ('Parallel')
Name_step1='Falcon Forward behavior';
Name_step2='Falcon Backward behavior';

%%
%Conversion output matrix from radians to degrees (for columns 2 , 4 et 5)
        deg=360./(2.*pi);
        ztransf=[1 0 0 0 0;0 deg 0 0 0;0 0 1 0 0;0 0 0 deg 0;0 0 0 0 deg];
        zdeg_1=[z_1]*[ztransf];
        r_1=length(z_1(:,1));
        
%%       
%Areas after initial conditions of Step 2 Next and Parallel Modes declaration : 
    %B. Step 2 in Next
            %C. Step 2 in Parallel
%A.There is only Step 1
%%
%Step 2

%Initial conditions Step 2
if Step2=='Y';        
    
switch Conf_step2
    case 'Parallel'
        T0_2=T0;
        T1_2=T1;
        
        alph0_2=alph0;%angle of attack initial in degrees (alph=a+f)
        u0_2 =u0 ; %initial position (0) 
        a0_2 =a0; %flight path angle in degrees Gamma
        b0_2=b0 ;%Pitch rate attitude in degrees/sec� Theta dot
        v0_2=v0;%initial airspeed in m/s
        
                %Transformation in radians
                alphr0_2=alph0_2.*pi./180;
                ar0_2=(a0_2*pi)/180;%flight path angle in radians 
                fr0_2=alphr0_2+ar0_2;% pitch attitude in radians/sec Theta
                br0_2=b0_2.*pi./180;%Pith rate attitude in radians/sec�
                
        Lb=-0.74; %0.4 if forward , -0.74 if backward :distance between the cgpilot and the axis of the control bar,if trim change 'Pil_act' 
        Pil_act='Active';% Choose Trim if the pilot totally remove his hands off the bar, or Active to make the pilot controlling the control bar or not.
        Pil_pos='Prone'; % Choose Stand or Prone
        
        Glider='Falcon';%Falcon or T2C
            
         
    case 'Next'
        
        alph0_2=(zdeg_1(r_1,2)-zdeg_1(r_1,4));%angle of attack initial in degrees (alph=a+f)
        u0_2 =zdeg_1(r_1,1) ; %initial position (0) 
        a0_2 =zdeg_1(r_1,4); %flight path angle in degrees Gamma
        b0_2=zdeg_1(r_1,5) ;%Pitch rate attitude in degrees/sec� Theta dot
        %b0_2=-90; %Pitch rate if gust
        v0_2=zdeg_1(r_1,3);%initial airspeed in m/s
        %v0_2=5;  % Airspeed if rear gust     
                
                %Transformation in radians
                alphr0_2=alph0_2.*pi./180;
                ar0_2=(a0_2.*pi)/180;%flight path angle in radians 
                fr0_2=alphr0_2+ar0_2;% pitch attitude in radians/sec Theta
                br0_2=b0_2.*pi./180;%Pith rate attitude in radians/sec�
        
        DT2 =8;   %How long Step 2 lasts
        T0_2=T1;
        T1_2=T0_2+DT2;
        
        Lb=-0.5; %0.4 if forward , -0.74 if backward :distance between the cgpilot and the axis of the control bar,if trim change 'Pil_act'
        Pil_act='Active';% Choose Trim , or Active to make the pilot controlling the control bar or not.
        Pil_pos='Prone'; % Choose Stand or Prone
end

[t_2,z_2] = ode45('Motions',T0_2:timespan:T1_2,[u0_2 fr0_2 v0_2 ar0_2 br0_2]); % Runge-Kutta 4th and 5th order

%Conversion output matrix from radians to degrees (for columns 2 , 4 et 5)
         ztransf_2=[1 0 0 0 0;0 deg 0 0 0;0 0 1 0 0;0 0 0 deg 0;0 0 0 0 deg];
         zdeg_2=[z_2]*[ztransf_2];
         r_2=length(z_2(:,1));
        

switch Conf_step2
                                 case 'Next'
                                 z=[z_1;z_2];
                                 zdeg=[zdeg_1;zdeg_2];
                                 t=[t_1;t_2];

                            %B.Projection on x and y of U ( thank to gamma)
                            Uz=z(:,1);%Vector U from Z
                            cosG=cos(z(:,4));
                            sinG=sin(z(:,4));

                            r=length(Uz);
                            dUz=zeros(1,r)';
                            for i=2:1:r
                            dUz(i)=Uz(i)-Uz(i-1);
                            end

                            ddUz=diag(dUz);% Diagonal for product

                            Ux=zeros(1,r)';
                            for i=2:r
                                Ux(i)=ddUz(i,i).*cosG(i-1)+Ux(i-1);
                            end

                            Uy=zeros(1,r)';
                            for i=2:r
                                Uy(i)=ddUz(i,i).*sinG(i-1)+Uy(i-1);
                            end

                            Uz=zeros(1,r)'; % Position Z constant in the model to plot 3D curve

                            Vz=z(:,3);
                            dVz=zeros(1,r);
                            for i=2:1:r;
                                dVz(i)=Vz(i)-Vz(i-1);
                            end

                            ddVz=diag(dVz);

                            Vx=zeros(1,r);
                            for i=2:r
                                Vx(i)=ddVz(i,i).*cosG(i)+Vx(i-1);
                            end

                            Vy=zeros(1,r);
                            for i=2:r
                                Vy(i)=ddVz(i,i).*sinG(i)+Vy(i-1);
                            end
                            %%
                            %B.Warning message if alpha out of the domain we know it.
                            Alpha_fig=(zdeg(:,2)-zdeg(:,4));% because alpha is not a natural output , so we build it to plot it
                            n=length(Alpha_fig);

                            msg='0';
                            for i=1:1:n
                               if Alpha_fig(i) > alphamax|Alpha_fig(i)<alphamin,
                              %disp('During the simulation, Alpha is out of the domain we knows the behavior law')
                              msg ='During the simulation, Alpha is out of the domain we knows the behavior law';
                               end   
                            end

                            if msg~='0',
                            warning(msg)
                            else 
                            disp('Alpha remains in the domain we know the behavior law')
                            end

                            %%
                            figure('Name','Dashboard','NumberTitle','off');
                            subplot(3,2,1);
                            plot(Ux,Uy,'r');
                            title('Trajectory');
                            xlabel('X(m)');
                            ylabel('Y(m)');

                            subplot(3,2,2)
                            plot(t,zdeg(:,3),'b')
                            xlabel('Time');
                            ylabel('Speed (m/s)');
                            title('Velocity as a function of time')

                            subplot(3,2,3)
                            plot(t,zdeg(:,2),'r')
                            xlabel('Time');
                            ylabel('Pitch attitude (deg)');
                            title('Pitch attitude as a function of time')

                            subplot(3,2,4)
                            plot(t,zdeg(:,5),'b')
                            xlabel('Time');
                            ylabel('Pitch rate attitude (deg/s)');
                            title('Pitch rate as a function of time')

                            subplot(3,2,5)
                            plot(t,zdeg(:,4),'g')
                            xlabel('Time');
                            ylabel('Flight path angle (deg)');
                            title('Flight path angle as a function of time')

                            subplot(3,2,6)
                            plot(t,Alpha_fig,'b');
                            ylim([alphamin alphamax+15]);
                            xlabel('Time')
                            ylabel('Alpha (deg)');
                            title('Angle of attack ');

                            %%
                            %B.3D Trajectory
                            %figure;
                            %plot3(Ux,Uz,Uy,'r')
                            %title('3D trajectory')
                            %axis([0 max(Ux) -1 1 min(Uy) 0])
                            %grid on
                            %grid minor
                            %view(50,26)
                            %xlabel('Horizontal axis')
                            %zlabel('Vertical axis')

                            %text(0,0,0,'Start Point!')
                            %text(max(Ux),0,min(Uy),'End point!')

                            %%
                            %B.TR5%
                            alpharef=Alpha_fig(r)

                            alpha_ut=flipdim(Alpha_fig,1);
                            t_ut=flipdim(t,1);
                            i=1;
                            if abs(alpharef-alpha_ut(r))<0.05*alpharef
                                Tr5=0;
                            else while abs(alpharef-alpha_ut(i))<0.05*abs(alpharef)
                                Tr5=t_ut(i);
                                i=i+1;
                                end
                            end

                            %%
                            %B.Alpha Tr5%
                            figure;
                            plot(t,Alpha_fig);
                            hold('on');
                            plot([0 max(t)],[alpharef alpharef],'r')
                            plot([0 max(t)],[0.95*alpharef 0.95*alpharef],'g')
                            plot([0 max(t)],[1.05*alpharef 1.05*alpharef],'g')
                            plot([Tr5 Tr5],[0 alpharef*1.05],'k')
                            text(Tr5,1,'\leftarrow Tr5%','HorizontalAlignment','left')
                            xlabel('Time');
                            ylabel('Alpha (deg)');
                            title('Angle of attack '); 
                            %%
                            %B.Trajectory with another background

                            img1 = imread('a_bird.jpg');
                            figure;
                            imagesc([0 max(Ux)], [min(Uy)-30 max(Uy)], flipdim(img1,1));
                            xlabel('Horizontal Displacement (m)');
                            ylabel('Vertical Displacement (m)');
                            title('Trajectory');
                            colormap(gray);
                            hold ('on')
                            plot(Ux,Uy,'b-*','linewidth',2);
                            set(gca,'ydir','normal');
                            daspect([1 1 1]);
                            %%        
                            %B.Output interesting to get :
                                    Tr5
                                    Cmo
                                    Glide_angle=-(Ux(r)-Ux(r-1))/(Uy(r)-Uy(r-1))  % Value taken from the two last points of the simulation
                                    Vstable=zdeg(r,3)
                                    Alph_stable=Alpha_fig(r)
                                    Gam_stable=zdeg(r,4)

                            %figure;
                            %comet3(Ux,Uy,0.1)
                            %daspect([1 1 1]);
                            %grid on
                            %axis([0 max(Ux) -10 10 min(Uy) 0])

                            if export=='Y'

                            if exist( filename,'file');
                             delete (filename)
                            end

                            traj=['Traject_',filename];
                            if exist( traj,'file');
                             delete (traj)
                            end   

                            col_header={'Time(s)','Flight path angle (deg)','Angle of Attack (Alpha)(deg)','Position on X axis (m)','Position on Y axis (m)','Pitch attitude (deg)','Speed (m/s)'}; %Row cell array (for column labels)
                            xlswrite(filename,col_header,'Feuil1','A1');
                            xlswrite(filename,t,'Feuil1','A2');
                            xlswrite(filename,zdeg(:,4),'Feuil1','B2');
                            xlswrite(filename,Alpha_fig,'Feuil1','C2');
                            xlswrite(filename,Ux,'Feuil1','D2');
                            xlswrite(filename,Uy,'Feuil1','E2');
                            xlswrite(filename,zdeg(:,2),'Feuil1','F2');
                            xlswrite(filename,zdeg(:,3),'Feuil1','G2');

                            col_header2={'Initial conditions','Speed0','Theta0','Theta_dot0','alpha0','Gamma0','Pilot act','Pilot position','Lb'}';
                            xlswrite(filename,col_header2,'Feuil1','I2');
                            col_header4={v0,zdeg(1,2),b0,alph0,a0,Pil_act,Pil_pos,Lb}';
                            xlswrite(filename,col_header4,'Feuil1','J3');
                            col_header3={'Position on X axis (m)','Position on Y axis (m)'};
                            xlswrite(traj,col_header2,'Feuil1','A1');
                            xlswrite(traj,Ux,'Feuil1','a2');
                            xlswrite(traj,Uy,'Feuil1','b2');
                            end
    
                                            case 'Parallel'
                                                    z=z_1;
                                                    zdeg=zdeg_1;
                                                    t=t_1;

                                        %C.Projection on x and y of U ( thank to gamma)
                                        Uz=z(:,1);%Vector U from Z
                                        cosG=cos(z(:,4));
                                        sinG=sin(z(:,4));

                                        r=length(Uz);
                                        dUz=zeros(1,r)';
                                        for i=2:1:r
                                        dUz(i)=Uz(i)-Uz(i-1);
                                        end

                                        ddUz=diag(dUz);% Diagonal for product

                                        Ux=zeros(1,r)';
                                        for i=2:r
                                            Ux(i)=ddUz(i,i).*cosG(i-1)+Ux(i-1);
                                        end

                                        Uy=zeros(1,r)';
                                        for i=2:r
                                            Uy(i)=ddUz(i,i).*sinG(i-1)+Uy(i-1);
                                        end

                                        Uz=zeros(1,r)'; % Position Z constant in the model to plot 3D curve

                                        Vz=z(:,3);
                                        dVz=zeros(1,r);
                                        for i=2:1:r;
                                            dVz(i)=Vz(i)-Vz(i-1);
                                        end

                                        ddVz=diag(dVz);

                                        Vx=zeros(1,r);
                                        for i=2:r
                                            Vx(i)=ddVz(i,i).*cosG(i)+Vx(i-1);
                                        end

                                        Vy=zeros(1,r);
                                        for i=2:r
                                            Vy(i)=ddVz(i,i).*sinG(i)+Vy(i-1);
                                        end

                                        %C.Warning message if alpha out of the domain we know it.
                                        Alpha_fig=(zdeg(:,2)-zdeg(:,4));% because alpha is not a natural output , so we build it to plot it
                                        n=length(Alpha_fig);

                                        msg='0';
                                        for i=1:1:n
                                           if Alpha_fig(i) > alphamax|Alpha_fig(i)<alphamin,
                                          %disp('During the simulation, Alpha from Step 1 is out of the domain we knows the behavior law')
                                          msg ='During the simulation, Alpha from Step 1 is out of the domain we knows the behavior law';
                                           end   
                                        end

                                        if msg~='0',
                                        warning(msg)
                                        else 
                                        disp('Alpha remains in the domain we know the behavior law in Step 1')
                                        end
                                         %C.Projection on x and y of U ( thank to gamma)
                                        Uz_2=z_2(:,1);%Vector U from Z
                                        cosG_2=cos(z_2(:,4));
                                        sinG_2=sin(z_2(:,4));

                                        r_2=length(Uz_2);
                                        dUz_2=zeros(1,r_2)';
                                        for i=2:1:r_2
                                        dUz_2(i)=Uz_2(i)-Uz_2(i-1);
                                        end

                                        ddUz_2=diag(dUz_2);% Diagonal for product

                                        Ux_2=zeros(1,r_2)';
                                        for i=2:r_2
                                            Ux_2(i)=ddUz_2(i,i).*cosG_2(i-1)+Ux_2(i-1);
                                        end

                                        Uy_2=zeros(1,r_2)';
                                        for i=2:r_2
                                            Uy_2(i)=ddUz_2(i,i).*sinG_2(i-1)+Uy_2(i-1);
                                        end

                                        Uz_2=zeros(1,r_2)'; % Position Z constant in the model to plot 3D curve

                                        Vz_2=z_2(:,3);
                                        dVz_2=zeros(1,r_2);
                                        for i=2:1:r_2;
                                            dVz_2(i)=Vz_2(i)-Vz_2(i-1);
                                        end

                                        ddVz_2=diag(dVz_2);

                                        Vx_2=zeros(1,r_2);
                                        for i=2:r_2
                                            Vx_2(i)=ddVz_2(i,i).*cosG_2(i)+Vx_2(i-1);
                                        end

                                        Vy_2=zeros(1,r_2);
                                        for i=2:r_2
                                            Vy_2(i)=ddVz_2(i,i).*sinG_2(i)+Vy_2(i-1);
                                        end
                                        %%
                                        %C.Warning message if alpha out of the domain we know it.
                                        Alpha_fig_2=(zdeg_2(:,2)-zdeg_2(:,4));% because alpha is not a natural output , so we build it to plot it
                                        n_2=length(Alpha_fig_2); %n_2=r_2 if everything is correct

                                        msg='0';
                                        for i=1:1:n_2
                                           if Alpha_fig_2(i) > alphamax|Alpha_fig_2(i)<alphamin,
                                          %disp('During the simulation, Alpha from Step 2 is out of the domain we knows the behavior law')
                                          msg ='During the simulation, Alpha from Step 2 is out of the domain we knows the behavior law';
                                           end   
                                        end

                                        if msg~='0',
                                        warning(msg)
                                        else 
                                        disp('Alpha remains in the domain we know the behavior law in Step 2')
                                        end

                                        %%
                                        figure('Name','Dashboard','NumberTitle','off');
                                        subplot(3,2,1);
                                        plot(Ux,Uy,'b');
                                        hold on
                                        plot(Ux_2,Uy_2,'r');
                                        legend(Name_step1,Name_step2)
                                        title('Trajectory');
                                        xlabel('X(m)');
                                        ylabel('Y(m)');

                                        subplot(3,2,2)
                                        plot(t,zdeg(:,3),'b')
                                        hold on
                                        plot(t_2,zdeg_2(:,3),'r');
                                        legend(Name_step1,Name_step2)
                                        xlabel('Time');
                                        ylabel('Speed (m/s)');
                                        title('Velocity as a function of time')

                                        subplot(3,2,3)
                                        plot(t,zdeg(:,2),'b')
                                        hold on
                                        plot(t_2,zdeg_2(:,2),'r');
                                        legend(Name_step1,Name_step2)
                                        xlabel('Time');
                                        ylabel('Pitch attitude (deg)');
                                        title('Pitch attitude as a function of time')

                                        subplot(3,2,4)
                                        plot(t,zdeg(:,5),'b')
                                        hold on
                                        plot(t_2,zdeg_2(:,5),'r');
                                        legend(Name_step1,Name_step2)
                                        xlabel('Time');
                                        ylabel('Pitch rate attitude (deg/s)');
                                        title('Pitch rate as a function of time')

                                        subplot(3,2,5)
                                        plot(t,zdeg(:,4),'b')
                                        hold on
                                        plot(t_2,zdeg_2(:,4),'r');
                                        legend(Name_step1,Name_step2)
                                        ylim([-90 +45])
                                        xlabel('Time');
                                        ylabel('Flight path angle (deg)');
                                        title('Flight path angle as a function of time')

                                        subplot(3,2,6)
                                        plot(t,Alpha_fig,'b');
                                        hold on
                                        plot(t_2,Alpha_fig_2,'r');
                                        legend(Name_step1,Name_step2)
                                        xlabel('Time')
                                        ylabel('Alpha (deg)');
                                        ylim([alphamin alphamax+15]);
                                        title('Angle of attack ');

                                        %C.TR5%
                                        alpharef_1=Alpha_fig(r_1)
                                        alpharef_2=Alpha_fig_2(r_2)

                                        alpha_ut_1=flipdim(Alpha_fig,1);
                                        alpha_ut_2=flipdim(Alpha_fig_2,1);

                                        t_ut_1=flipdim(t_1,1);
                                        i=1;
                                        if abs(alpharef_1-alpha_ut_1(r))<0.05*alpharef_1
                                            Tr5_1=0;
                                        else while abs(alpharef_1-alpha_ut_1(i))<0.05*abs(alpharef_1)
                                            Tr5_1=t_ut_1(i);
                                            i=i+1;
                                            end
                                        end

                                        t_ut_2=flipdim(t_2,1);
                                        i=1;
                                        if abs(alpharef_2-alpha_ut_2(r_2))<0.05*alpharef_2
                                            Tr5_2=0;
                                                                       
                                        else while abs(alpharef_2-alpha_ut_2(i))<0.05*abs(alpharef_2)
                                            Tr5_2=t_ut_2(i);
                                            i=i+1;
                                            end
                                        end
                                        
                                        

                                        %%
                                        %C.Alpha Tr5%
                                        figure;
                                        plot(t,Alpha_fig);
                                        hold('on');
                                        plot([0 max(t_1)],[alpharef_1 alpharef_1],'r')
                                        plot([0 max(t_1)],[0.95*alpharef_1 0.95*alpharef_1],'g')
                                        plot([0 max(t_1)],[1.05*alpharef_1 1.05*alpharef_1],'g')
                                        plot([Tr5_1 Tr5_1],[0 alpharef_1*1.05],'k')
                                        text(Tr5_1,1,'\leftarrow Tr5%_1','HorizontalAlignment','left')
                                        plot(t_2,Alpha_fig_2)
                                        plot([0 max(t_2)],[alpharef_2 alpharef_2],'r')
                                        plot([0 max(t_2)],[0.95*alpharef_2 0.95*alpharef_2],'g')
                                        plot([0 max(t_2)],[1.05*alpharef_2 1.05*alpharef_2],'g')
                                        plot([Tr5_2 Tr5_2],[0 alpharef_2*1.05],'k')
                                        text(Tr5_2,1,'\leftarrow Tr5%_2','HorizontalAlignment','left')
                                        xlabel('Time');
                                        ylabel('Alpha (deg)');
                                        title('Angle of attack '); 

                                        %%
                                        %C.Trajectory with another background

                                        img1 = imread('a_bird.jpg');
                                        figure;
                                        imagesc([0 max(Ux)], [min(Uy)-30 max(Uy)], flipdim(img1,1));
                                        xlabel('Horizontal Displacement (m)');
                                        ylabel('Vertical Displacement (m)');
                                        title('Trajectory');
                                        colormap(gray);
                                        hold ('on')
                                        plot(Ux,Uy,'b-*','linewidth',2);
                                        plot(Ux_2,Uy_2,'r-*','linewidth',2);
                                        legend(Name_step1,Name_step2)
                                        set(gca,'ydir','normal');
                                        daspect([1 1 1]);
                                        
                                        %C.3D Trajectory
                                        %figure;
                                        %plot3(Ux,Uz,Uy,'b')
                                        %hold ('on')
                                        %plot3(Ux_2,Uz_2,Uy_2,'r')
                                        %title('3D trajectory')
                                        %axis([0 max(Ux) -1 1 min(Uy) 0])
                                        %grid on
                                        %grid minor
                                        %view(50,26)
                                        %xlabel('Horizontal axis')
                                        %zlabel('Vertical axis')

                                        %text(0,0,0,'Start Point!')
                                        %text(max(Ux),0,min(Uy),'End point!')


                                        %C.Output interesting to get :
                                                Tr5_1
                                                Tr5_2   
                                                Cmo
                                                Glide_angle_1=-(Ux(r_1)-Ux(r_1-1))/(Uy(r_1)-Uy(r_1-1))  % Value taken from the two last points of the simulation
                                                Glide_angle_2=-(Ux_2(r_2)-Ux_2(r_2-1))/(Uy_2(r_2)-Uy_2(r_2-1))  % Value taken from the two last points of the simulation
                                                Vstable_1=zdeg(r_1,3)
                                                Vstable_2=zdeg_2(r_2,3)
                                                Alph_stable_1=Alpha_fig(r_1)
                                                Alph_stable_2=Alpha_fig_2(r_2)
                                                Gam_stable_1=zdeg(r_1,4)
                                                Gam_stable_2=zdeg_2(r_2,4)
end         
end

if Step2=='N'
%A.Projection on x and y of U ( thank to gamma)
Uz=z_1(:,1);%Vector U from Z
cosG=cos(z_1(:,4));
sinG=sin(z_1(:,4));

r=length(Uz);
dUz=zeros(1,r)';
for i=2:1:r
dUz(i)=Uz(i)-Uz(i-1);
end

ddUz=diag(dUz);% Diagonal for product

Ux=zeros(1,r)';
for i=2:r
    Ux(i)=ddUz(i,i).*cosG(i-1)+Ux(i-1);
end

Uy=zeros(1,r)';
for i=2:r
    Uy(i)=ddUz(i,i).*sinG(i-1)+Uy(i-1);
end

Uz=zeros(1,r)'; % Position Z constant in the model to plot 3D curve

Vz=z_1(:,3);
dVz=zeros(1,r);
for i=2:1:r;
    dVz(i)=Vz(i)-Vz(i-1);
end

ddVz=diag(dVz);

Vx=zeros(1,r);
for i=2:r
    Vx(i)=ddVz(i,i).*cosG(i)+Vx(i-1);
end

Vy=zeros(1,r);
for i=2:r
    Vy(i)=ddVz(i,i).*sinG(i)+Vy(i-1);
end
%%
%A.Warning message if alpha out of the domain we know it.
Alpha_fig=(zdeg_1(:,2)-zdeg_1(:,4));% because alpha is not a natural output , so we build it to plot it
n=length(Alpha_fig);

msg='0';
for i=1:1:n
   if Alpha_fig(i) > alphamax|Alpha_fig(i)<alphamin,
  %disp('During the simulation, Alpha is out of the domain we knows the behavior law')
  msg ='During the simulation, Alpha is out of the domain we knows the behavior law';
   end   
end

if msg~='0',
warning(msg)
else 
disp('Alpha remains in the domain we know the behavior law')
end

%%
figure('Name','Dashboard','NumberTitle','off');
subplot(3,2,1);
plot(Ux,Uy,'r');
title('Trajectory');
xlabel('X(m)');
ylabel('Y(m)');

subplot(3,2,2)
plot(t_1,zdeg_1(:,3),'b')
xlabel('Time');
ylabel('Speed (m/s)');
title('Velocity as a function of time')

subplot(3,2,3)
plot(t_1,zdeg_1(:,2),'r')
xlabel('Time');
ylabel('Pitch attitude (deg)');
title('Pitch attitude as a function of time')

subplot(3,2,4)
plot(t_1,zdeg_1(:,5),'b')
xlabel('Time');
ylabel('Pitch rate attitude (deg/s)');
ylim=([-90 90]);
title('Pitch rate as a function of time')

subplot(3,2,5)
plot(t_1,zdeg_1(:,4),'g')
AXIS([0 t_1(r_1) -90 45])
xlabel('Time');
ylabel('Flight path angle (deg)');
title('Flight path angle as a function of time')

subplot(3,2,6)
plot(t_1,Alpha_fig,'b');
AXIS([0 t_1(r_1) alphamin alphamax+15])
%ylim([alphamin alphamax+15]);
xlabel('Time')
ylabel('Alpha (deg)');
title('Angle of attack ');

%%
%A.3D Trajectory
%figure;
%plot3(Ux,Uz,Uy,'r')
%title('3D trajectory')
%axis([0 max(Ux) -1 1 min(Uy) 0])
%grid on
%grid minor
%view(50,26)
%xlabel('Horizontal axis')
%zlabel('Vertical axis')

%text(0,0,0,'Start Point!')
%text(max(Ux),0,min(Uy),'End point!')

%%
%A.TR5%
alpharef=Alpha_fig(r)

alpha_ut=flipdim(Alpha_fig,1);
t_ut=flipdim(t_1,1);
i=1;
if abs(alpharef-alpha_ut(r))<0.05*alpharef
    Tr5=0;
else while abs(alpharef-alpha_ut(i))<0.05*abs(alpharef)
    Tr5=t_ut(i);
    i=i+1;
    end
end

%%
%A.Alpha Tr5%
figure;
plot(t_1,Alpha_fig);
hold('on');
plot([0 max(t_1)],[alpharef alpharef],'r')
plot([0 max(t_1)],[0.95*alpharef 0.95*alpharef],'g')
plot([0 max(t_1)],[1.05*alpharef 1.05*alpharef],'g')
plot([Tr5 Tr5],[0 alpharef*1.05],'k')
text(Tr5,1,'\leftarrow Tr5%','HorizontalAlignment','left')
xlabel('Time');
ylabel('Alpha (deg)');
title('Angle of attack '); 
%%
%A.Trajectory with another background

img1 = imread('a_bird.jpg');
figure;
imagesc([0 max(Ux)], [min(Uy)-30 max(Uy)], flipdim(img1,1));
xlabel('Horizontal Displacement (m)');
ylabel('Vertical Displacement (m)');
title('Trajectory');
colormap(gray);
hold ('on')
plot(Ux,Uy,'b-*','linewidth',2);
set(gca,'ydir','normal');
daspect([1 1 1]);
%%        
%A.Output interesting to get :
        Tr5
        Cmo
        Glide_angle=-(Ux(r)-Ux(r-1))/(Uy(r)-Uy(r-1))  % Value taken from the two last points of the simulation
        Vstable=zdeg_1(r,3)
        Alph_stable=Alpha_fig(r)
        Gam_stable=zdeg_1(r,4)

%figure;
%comet3(Ux,Uy,0.1)
%daspect([1 1 1]);
%grid on
%axis([0 max(Ux) -10 10 min(Uy) 0])

%A.Export in spreadsheet
if export=='Y'

if exist( filename,'file');
 delete (filename)
end

traj=['Traject_',filename];
if exist( traj,'file');
 delete (traj)
end   

col_header={'Time(s)','Flight path angle (deg)','Angle of Attack (Alpha)(deg)','Position on X axis (m)','Position on Y axis (m)','Pitch attitude (deg)','Speed (m/s)'}; %Row cell array (for column labels)
xlswrite(filename,col_header,'Feuil1','A1');
xlswrite(filename,t_1,'Feuil1','A2');
xlswrite(filename,zdeg_1(:,4),'Feuil1','B2');
xlswrite(filename,Alpha_fig,'Feuil1','C2');
xlswrite(filename,Ux,'Feuil1','D2');
xlswrite(filename,Uy,'Feuil1','E2');
xlswrite(filename,zdeg_1(:,2),'Feuil1','F2');
xlswrite(filename,zdeg_1(:,3),'Feuil1','G2');

col_header2={'Initial conditions','Speed0','Theta0','Theta_dot0','alpha0','Gamma0','Pilot act','Pilot position','Lb'}';
xlswrite(filename,col_header2,'Feuil1','I2');
col_header4={v0,zdeg_1(1,2),b0,alph0,a0,Pil_act,Pil_pos,Lb}';
xlswrite(filename,col_header4,'Feuil1','J3');
col_header3={'Position on X axis (m)','Position on Y axis (m)'};
xlswrite(traj,col_header2,'Feuil1','A1');
xlswrite(traj,Ux,'Feuil1','a2');
xlswrite(traj,Uy,'Feuil1','b2');
end
end

