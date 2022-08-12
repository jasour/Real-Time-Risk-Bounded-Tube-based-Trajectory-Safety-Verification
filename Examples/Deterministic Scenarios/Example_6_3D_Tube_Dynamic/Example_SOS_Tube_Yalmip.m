%% SOS based continuous-time safety verification in Spotless
clc; clear;close all, warning off

% time
sdpvar t

% trajectory (x(t)=Px, y(t)=Py, z(t)=Pz)
Px=t; % trajectory x(t)
Py=t; % trajectory y(t)
Pz=((t-5)^4 + 2*(t-5)^3 - 15*(t-5)^2 - 12*(t-5) + 36)/20; % trajectory z(t)

% start and final time, i.e., t in [t0 tf]
t0=0;tf=9; 

% size of the tube
R=0.4;

% Obstacle: g(x1,x2,x3) <=0  ---- > Being Safe: >=0 
    Safe= @(x1,x2,x3,t) ((x1-2)/1)^2+((x2-2)/2)^2+((x3-5+t/1.5)/2)^2-1^2;%  Example 1: status 1: Trajectory is safe


% SOS relaxation order
d=10;

% Obs(x) <=0 ---> Safe(x)>=0 --->Safe(x(t)+xt,y(t)+yt)>=0 for all t0 =<t=< tf and (xt,yt) in {R^2-xt^2-yt^2} 
status=func_3D_SOS_Tube_yalmip(Safe,Px,Py,Pz,t0,tf,R,d);


%% visualization
[x1,x2,x3]=meshgrid([-3:0.1:7],[-3:0.1:7],[-3:0.1:7]);
l=10; n=100;x=linspace(-l,l,n);y=linspace(-l,l,n);z=linspace(-l,l,n);
[X,Y,Z] = sphere;
s=0;
for tt=1:1.1:tf
s=s+1;   
% trajectory
t=[t0:0.1:tt]; 
Px=t; % trajectory x(t)
Py=t; % trajectory y(t)
Pz=((t-5).^4 + 2*(t-5).^3 - 15*(t-5).^2 - 12*(t-5) + 36)/20; % trajectory z(t)
subplot(2,4,s);hold on
% obs
t=tt;Safe=  ((x1-2)/1).^2+((x2-2)/2).^2+((x3-5+t/1.5)/2).^2-1^2;%  Example 1: status 1: Trajectory is safe
p=patch(isosurface(x1,x2,x3,Safe,0));
set(p,'FaceColor','red','EdgeColor','none','FaceAlpha',0.7);hold on
% tube
for i=1:size(Px,2); surf(R*X+Px(i),R*Y+Py(i),R*Z+Pz(i),'FaceColor','blue','EdgeColor','none','FaceAlpha',0.1); end
surf(R*X+Px(end),R*Y+Py(end),R*Z+Pz(end),'FaceColor','yellow','EdgeColor','none')
camlight; lighting gouraud; 
% traj
plot3(Px,Py,Pz,'r--','LineWidth',2);grid on; hold on  
xlim([0 10]);ylim([0 10]);zlim([-5 10])
view([15 15])
title(sprintf('t=%0.1f',tt))        
end



