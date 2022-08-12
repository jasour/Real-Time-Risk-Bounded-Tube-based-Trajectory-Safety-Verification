%% SOS based continuous-time safety verification in Spotless
clc; clear;close all, warning off

% time
sdpvar t

% trajectory (x(t)=Px, y(t)=Py)
Px=t; % trajectory x(t)
Py=((t-5)^4 + 2*(t-5)^3 - 15*(t-5)^2 - 12*(t-5) + 36)/20; % trajectory y(t)

% start and final time, i.e., t in [t0 tf]
t0=0;tf=9; 

% size of the tube
R=0.4; 

% Obstacle: g(x) <=0  ---- > Being Safe: >=0 
     Safe= @(x1,x2,t) ((x1-2)/1)^2+((x2-5+t/1.5)/2)^2-1^2;%  Example 1: status 1: Trajectory is safe

% SOS relaxation order
d=10;

% Obs(x) <=0 ---> Safe(x)>=0 --->Safe(x(t)+xt,y(t)+yt)>=0 for all t0 =<t=< tf and (xt,yt) in {R^2-xt^2-yt^2} 
status=func_2D_SOS_Tube_yalmip(Safe,Px,Py,t0,tf,R,d);


%% visualization
[x1,x2]=meshgrid([-3:0.1:7],[-3:0.1:7]);
s=0;
for tt=0.5:0.5:tf
s=s+1;
t=[t0:0.1:tt];    
Px=t; % trajectory x(t)
Py=((t-5).^4 + 2*(t-5).^3 - 15*(t-5).^2 - 12*(t-5) + 36)/20; % trajectory y(t)
subplot(3,6,s);hold on
th = 0:pi/50:2*pi; for i=1:size(Px,2); plot(R * cos(th) + Px(i), R * sin(th) + Py(i),'b'); end
fill(R * cos(th) + Px(end),R * sin(th) + Py(end),'g')
plot(Px,Py,'r--','LineWidth',2);grid on; hold on  
t=tt; Safe = ((x1-2)/1).^2+((x2-5+t/1.5)/2).^2-1^2; contour(x1,x2,Safe,[0 0],'r');
%axis square
title(sprintf('t=%0.1f',tt))        
end

