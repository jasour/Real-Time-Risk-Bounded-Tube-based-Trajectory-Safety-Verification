%% SOS based continuous-time safety verification in Spotless
clc; clear;close all, warning off

% time
t = msspoly('t',1); 

% trajectory (x(t)=Px, y(t)=Py, z(t)=Pz)
Px=t; % trajectory x(t)
Py=t; % trajectory y(t)
Pz=((t-5)^4 + 2*(t-5)^3 - 15*(t-5)^2 - 12*(t-5) + 36)/20; % trajectory z(t)

% start and final time, i.e., t in [t0 tf]
t0=0;tf=9; 

% size of the tube
R=0.4;

% Risk Contours Delta=0.01, Being Safe: >=0 
% C * A_SOS_Cons_1>=0 (C is a alrge positive number to avoid numerical issues) and A_SOS_Cons_2>=0
safe_1= @(x1,x2,x3,t) (16439709*x1)/350000 + (33691053*x2)/2800000 + (16435551*x3)/1400000 - (99*x1^2*x2^2)/200 - (99*x1^2*x3^2)/200 - (99*x2^2*x3^2)/800 - (4059*x1*x2)/500 - (198*x1*x3)/25 - (4059*x2*x3)/2000 + (x1^2 - 4*x1 + x2^2/4 - (41*x2)/40 + x3^2/4 - x3 + 99601/16800)^2 + (99*x1*x2^2)/50 + (4059*x1^2*x2)/2000 + (99*x1*x3^2)/50 + (99*x1^2*x3)/50 + (4059*x2*x3^2)/8000 + (99*x2^2*x3)/200 - (38615709*x1^2)/1400000 + (198*x1^3)/25 - (890373*x2^2)/224000 - (99*x1^4)/100 + (4059*x2^3)/8000 - (21979551*x3^2)/5600000 - (99*x2^4)/1600 + (99*x3^3)/200 - (99*x3^4)/1600 - 389925611097/11200000000;
Safe_1= @(x1,x2,x3,t) safe_1(x1,x2,x3,t)*1000; % C * A_SOS_Cons_1>=0 where C=1000
Safe_2= @(x1,x2,x3,t) x1^2 - 4*x1 + x2^2/4 - (41*x2)/40 + x3^2/4 - x3 + 99601/16800;

% SOS relaxation order
d=2;

% Obs(x) <=0 ---> Safe(x)>=0 --->Safe(x(t)+xt,y(t)+yt)>=0 for all t0 =<t=< tf and (xt,yt) in {R^2-xt^2-yt^2} 
status_1=func_3D_SOS_Tube_spotless(Safe_1,Px,Py,Pz,t0,tf,R,d);
status_2=func_3D_SOS_Tube_spotless(Safe_2,Px,Py,Pz,t0,tf,R,d);
clc; 
if status_1 ==1 & status_2 ==1
    status =1;   display('Trajectory is safe.')
else
    status =0;   display('Trajectory is NOT safe.')
end

%% visualization
l=10; n=200;x=linspace(-l,l,n);y=linspace(-l,l,n);z=linspace(-l,l,n);
[x1,x2,x3]=meshgrid(x,y,z);
safe_1= (16439709.*x1)/350000 + (33691053.*x2)/2800000 + (16435551.*x3)/1400000 - (99.*x1.^2.*x2.^2)/200 - (99.*x1.^2.*x3.^2)/200 - (99.*x2.^2.*x3.^2)/800 - (4059.*x1.*x2)/500 - (198.*x1.*x3)/25 - (4059.*x2.*x3)/2000 + (x1.^2 - 4.*x1 + x2.^2/4 - (41.*x2)/40 + x3.^2/4 - x3 + 99601/16800).^2 + (99.*x1.*x2.^2)/50 + (4059.*x1.^2.*x2)/2000 + (99.*x1.*x3.^2)/50 + (99.*x1.^2.*x3)/50 + (4059.*x2.*x3.^2)/8000 + (99.*x2.^2.*x3)/200 - (38615709.*x1.^2)/1400000 + (198.*x1.^3)/25 - (890373.*x2.^2)/224000 - (99.*x1.^4)/100 + (4059.*x2.^3)/8000 - (21979551.*x3.^2)/5600000 - (99.*x2.^4)/1600 + (99.*x3.^3)/200 - (99.*x3.^4)/1600 - 389925611097/11200000000;
p=patch(isosurface(x1,x2,x3,safe_1,0));
set(p,'FaceColor','red','EdgeColor','none','FaceAlpha',0.7);
%safe_2= x1.^2 - 4.*x1 + x2.^2/4 - (41.*x2)/40 + x3.^2/4 - x3 + 99601/16800;
%p=patch(isosurface(x1,x2,x3,safe_2,0));
%set(p,'FaceColor','red','EdgeColor','none','FaceAlpha',0.3);
camlight; lighting gouraud; hold on
% trajectory
t=[t0:0.05:tf];
Px=t; % trajectory x(t)
Py=t; % trajectory y(t)
Pz=((t-5).^4 + 2*(t-5).^3 - 15*(t-5).^2 - 12*(t-5) + 36)/20; % trajectory z(t)
plot3(Px(1),Py(1),Pz(1),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
plot3(Px(end),Py(end),Pz(end),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
plot3(Px,Py,Pz,'--','LineWidth',2);grid on; 
view([15 15])
% tube
[X,Y,Z] = sphere;
for i=1:size(Px,2); surf(R*X+Px(i),R*Y+Py(i),R*Z+Pz(i),'FaceColor','blue','EdgeColor','none','FaceAlpha',0.1); end
plot3(Px,Py,Pz,'--','LineWidth',2);grid on; 
camlight; lighting gouraud; 





