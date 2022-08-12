%% SOS-based Continuous-Time Safety Verification in Spotless
clc; clear;close all, warning off

% time
t = msspoly('t',1); 

% trajectory (x(t)=Px, y(t)=Py, z(t)=Pz)
Px=t; % trajectory x(t)
Py=t; % trajectory y(t)
Pz=((t-5)^4 + 2*(t-5)^3 - 15*(t-5)^2 - 12*(t-5) + 36)/20; % trajectory z(t)

% start and final time, i.e., t in [t0 tf]
t0=0;tf=9; 

% Obstacle: g(x1,x2,x3) <=0  ---- > Being Safe: >=0 
   Safe= @(x1,x2,x3,t) ((x1-2)/1)^2+((x2-2)/2)^2+((x3-2)/2)^2-1^2;%  Example 1: status 1: Trajectory is safe
%  Safe= @(x1,x2,x3,t) ((x1-3.5)/1)^2+((x2-2)/2)^2+((x3-1)/2)^2-1^2;%  Example 2: status 1: Trajectory is NOT safe

% SOS relaxation order
d=2;

% Obs(x) <=0 ---> Safe(x)>=0 --->Safe(x(t),y(t))>=0 for all t0 =<t=< tf
status=func_3D_SOS_spotless(Safe,Px,Py,Pz,t0,tf,d) ;


%% visualization
l=10; n=100;x=linspace(-l,l,n);y=linspace(-l,l,n);z=linspace(-l,l,n);
[x1,x2,x3]=meshgrid(x,y,z);
  Safe=  ((x1-2)/1).^2+((x2-2)/2).^2+((x3-2)/2).^2-1^2;%  Example 1: status 1: Trajectory is safe
% Safe = ((x1-3.5)/1).^2+((x2-2)/2).^2+((x3-1)/2).^2-1^2; % Example 2: status 1: Trajectory is NOT safe
p=patch(isosurface(x1,x2,x3,Safe,0));
set(p,'FaceColor','red','EdgeColor','none','FaceAlpha',0.7);
camlight; lighting gouraud; hold on
% trajectory
PPx=[];PPy=[]; PPz=[];
for tt=[t0:0.1:tf];  PPx=[PPx,double(subs(Px,t,tt))]; PPy=[PPy,double(subs(Py,t,tt))]; PPz=[PPz,double(subs(Pz,t,tt))];end
plot3(PPx(1),PPy(1), PPz(1),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
plot3(PPx(end),PPy(end),PPz(end),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
plot3(PPx,PPy,PPz,'--','LineWidth',2);grid on; 
view([15 15])
xlim([0 10]);ylim([0 10])
