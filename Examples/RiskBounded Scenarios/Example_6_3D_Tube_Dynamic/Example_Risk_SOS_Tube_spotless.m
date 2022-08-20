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
safe_1= @(x1,x2,x3,t) (10329517*t)/280000 + (30992709*x1)/350000 + (63524703*x2)/2800000 + (30988551*x3)/560000 - (11*t^2*x1^2)/50 - (11*t^2*x2^2)/200 - (33*t^2*x3^2)/200 - (99*x1^2*x2^2)/200 - (99*x1^2*x3^2)/200 - (99*x2^2*x3^2)/800 - (66*t*x1)/5 - (1353*t*x2)/400 - (21879517*t*x3)/1400000 - (4059*x1*x2)/500 - (99*x1*x3)/5 - (4059*x2*x3)/800 + (33*t*x1^2)/10 + (22*t^2*x1)/25 + (33*t*x2^2)/40 + (451*t^2*x2)/2000 + (99*t*x3^2)/40 + (33*t^2*x3)/20 - (33*t*x3^3)/200 - (11*t^3*x3)/150 + (99*x1*x2^2)/50 + (4059*x1^2*x2)/2000 + (99*x1*x3^2)/50 + (99*x1^2*x3)/20 + (4059*x2*x3^2)/8000 + (99*x2^2*x3)/80 - (21879517*t^2)/4200000 + (11*t^3)/30 - (11*t^4)/900 - (53168709*x1^2)/1400000 + (198*x1^3)/25 - (1472493*x2^2)/224000 - (99*x1^4)/100 + (4059*x2^3)/8000 - (65638551*x3^2)/5600000 - (99*x2^4)/1600 + (99*x3^3)/80 - (99*x3^4)/1600 + (t^2/9 + (t*x3)/3 - (5*t)/3 + x1^2 - 4*x1 + x2^2/4 - (41*x2)/40 + x3^2/4 - (5*x3)/2 + 187801/16800)^2 - (33*t*x1^2*x3)/50 - (33*t*x2^2*x3)/200 + (66*t*x1*x3)/25 + (1353*t*x2*x3)/2000 - 1385831753097/11200000000; 
Safe_1= @(x1,x2,x3,t) safe_1(x1,x2,x3,t)*1000000; % C * A_SOS_Cons_1>=0 where C=1000000
Safe_2= @(x1,x2,x3,t) t^2/9 + (t*x3)/3 - (5*t)/3 + x1^2 - 4*x1 + x2^2/4 - (41*x2)/40 + x3^2/4 - (5*x3)/2 + 187801/16800;


% SOS relaxation order
d=2;

% Safe(x)>=0 --->Safe(x(t),y(t))>=0 for all t0 =<t=< tf
status_1=func_3D_SOS_Tube_spotless(Safe_1,Px,Py,Pz,t0,tf,R,d);
status_2=func_3D_SOS_Tube_spotless(Safe_2,Px,Py,Pz,t0,tf,R,d);
clc; 
if status_1 ==1 & status_2 ==1
    status =1;   display('Trajectory is safe.')
else
    status =0;   display('Trajectory is NOT safe.')
end


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
t=tt;Safe=(10329517.*t)/280000 + (30992709.*x1)/350000 + (63524703.*x2)/2800000 + (30988551.*x3)/560000 - (11.*t.^2.*x1.^2)/50 - (11.*t.^2.*x2.^2)/200 - (33.*t.^2.*x3.^2)/200 - (99.*x1.^2.*x2.^2)/200 - (99.*x1.^2.*x3.^2)/200 - (99.*x2.^2.*x3.^2)/800 - (66.*t.*x1)/5 - (1353.*t.*x2)/400 - (21879517.*t.*x3)/1400000 - (4059.*x1.*x2)/500 - (99.*x1.*x3)/5 - (4059.*x2.*x3)/800 + (33.*t.*x1.^2)/10 + (22.*t.^2.*x1)/25 + (33.*t.*x2.^2)/40 + (451.*t.^2.*x2)/2000 + (99.*t.*x3.^2)/40 + (33.*t.^2.*x3)/20 - (33.*t.*x3.^3)/200 - (11.*t.^3.*x3)/150 + (99.*x1.*x2.^2)/50 + (4059.*x1.^2.*x2)/2000 + (99.*x1.*x3.^2)/50 + (99.*x1.^2.*x3)/20 + (4059.*x2.*x3.^2)/8000 + (99.*x2.^2.*x3)/80 - (21879517.*t.^2)/4200000 + (11.*t.^3)/30 - (11.*t.^4)/900 - (53168709.*x1.^2)/1400000 + (198.*x1.^3)/25 - (1472493.*x2.^2)/224000 - (99.*x1.^4)/100 + (4059.*x2.^3)/8000 - (65638551.*x3.^2)/5600000 - (99.*x2.^4)/1600 + (99.*x3.^3)/80 - (99.*x3.^4)/1600 + (t.^2/9 + (t.*x3)/3 - (5.*t)/3 + x1.^2 - 4.*x1 + x2.^2/4 - (41.*x2)/40 + x3.^2/4 - (5.*x3)/2 + 187801/16800).^2 - (33.*t.*x1.^2.*x3)/50 - (33.*t.*x2.^2.*x3)/200 + (66.*t.*x1.*x3)/25 + (1353.*t.*x2.*x3)/2000 - 1385831753097/11200000000; 
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



