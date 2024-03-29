

%% SOS based continuous-time safety verification in Yalmip
clc; clear;close all, warning off

% time
sdpvar t

% trajectory (x(t)=Px, y(t)=Py)
Px=t; % trajectory x(t)
Py=((t-5)^4 + 2*(t-5)^3 - 15*(t-5)^2 - 12*(t-5) + 36)/20; % trajectory y(t)

% start and final time, i.e., t in [t0 tf]
t0=0;tf=9; 


% Obstacle: g(x) <=0  ---- > Being Safe: >=0 
  Safe= @(x1,x2,t) ((x1-2)/1)^2+((x2-2)/2)^2-1^2;%  Example 1: status 1: Trajectory is safe
%  Safe= @(x1,x2,t) ((x1-3)/1)^2+((x2-2)/2)^2-1^2;%  Example 2: status 0  Trajectory is NOT safe

% SOS relaxation order
d=2;

% Obs(x) <=0 ---> Safe(x)>=0 --->Safe(x(t),y(t))>=0 for all t0 =<t=< tf
status=func_2D_SOS_yalmip(Safe,Px,Py,t0,tf,d);


%% visualization
% obs/safe
hold off; syms x1 x2;fcontour(Safe(x1,x2,0),'LevelList',[0 0],'LineWidth',3,'LineColor','r');hold on
% trajectory
t=[t0:0.1:tf];
Px=t; % trajectory x(t)
Py=((t-5).^4 + 2*(t-5).^3 - 15*(t-5).^2 - 12*(t-5) + 36)/20; % trajectory y(t)
plot(Px(1),Py(1),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
plot(Px(end),Py(end),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
plot(Px,Py,'--','LineWidth',2);grid on; 
