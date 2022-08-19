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

% Risk Contours Delta=0.01, Being Safe: >=0 
% C * A_SOS_Cons_1>=0 (C is a alrge positive number to avoid numerical issues) and A_SOS_Cons_2>=0
safe_1= @(x1,x2,t) (956903189*t)/28000000 + (7106979*x1)/87500 + (2870709567*x2)/56000000 - (11*t^2*x1^2)/50 - (33*t^2*x2^2)/200 - (99*x1^2*x2^2)/200 - (3333*t*x1)/250 - (5314111*t*x2)/350000 - (9999*x1*x2)/500 + (3333*t*x1^2)/1000 + (22*t^2*x1)/25 + (9999*t*x2^2)/4000 + (3333*t^2*x2)/2000 - (33*t*x2^3)/200 - (11*t^3*x2)/150 + (99*x1*x2^2)/50 + (9999*x1^2*x2)/2000 + (t^2/9 + (t*x2)/3 - (101*t)/60 + x1^2 - 4*x1 + x2^2/4 - (101*x2)/40 + 215321/21000)^2 - (5314111*t^2)/1050000 + (1111*t^3)/3000 - (11*t^4)/900 - (12650979*x1^2)/350000 + (198*x1^3)/25 - (15942333*x2^2)/1400000 - (99*x1^4)/100 + (9999*x2^3)/8000 - (99*x2^4)/1600 - (33*t*x1^2*x2)/50 + (66*t*x1*x2)/25 - 582954421137/5600000000;
Safe_1= @(x1,x2,t) safe_1(x1,x2,t)*1000; % C * A_SOS_Cons_1>=0 where C=1000
Safe_2= @(x1,x2,t) t^2/9 + (t*x2)/3 - (101*t)/60 + x1^2 - 4*x1 + x2^2/4 - (101*x2)/40 + 215321/21000;


% SOS relaxation order
d=10;

% Safe(x)>=0 --->Safe(x(t),y(t))>=0 for all t0 =<t=< tf
status_1=func_2D_SOS_Tube_yalmip(Safe_1,Px,Py,t0,tf,R,d);
status_2=func_2D_SOS_Tube_yalmip(Safe_2,Px,Py,t0,tf,R,d);
clc; 
if status_1 ==1 & status_2 ==1
    status =1;   display('Trajectory is safe.')
else
    status =0;   display('Trajectory is NOT safe.')
end

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
t=tt; safe_1 = (956903189.*t)/28000000 + (7106979.*x1)/87500 + (2870709567.*x2)/56000000 - (11.*t.^2.*x1.^2)/50 - (33.*t.^2.*x2.^2)/200 - (99.*x1.^2.*x2.^2)/200 - (3333.*t.*x1)/250 - (5314111.*t.*x2)/350000 - (9999.*x1.*x2)/500 + (3333.*t.*x1.^2)/1000 + (22.*t.^2.*x1)/25 + (9999.*t.*x2.^2)/4000 + (3333.*t.^2.*x2)/2000 - (33.*t.*x2.^3)/200 - (11.*t.^3.*x2)/150 + (99.*x1.*x2.^2)/50 + (9999.*x1.^2.*x2)/2000 + (t.^2/9 + (t.*x2)/3 - (101.*t)/60 + x1.^2 - 4.*x1 + x2.^2/4 - (101.*x2)/40 + 215321/21000).^2 - (5314111.*t.^2)/1050000 + (1111.*t.^3)/3000 - (11.*t.^4)/900 - (12650979.*x1.^2)/350000 + (198.*x1.^3)/25 - (15942333.*x2.^2)/1400000 - (99.*x1.^4)/100 + (9999.*x2.^3)/8000 - (99.*x2.^4)/1600 - (33.*t.*x1.^2.*x2)/50 + (66.*t.*x1.*x2)/25 - 582954421137/5600000000; 
contour(x1,x2,safe_1,[0 0],'r');
%axis square
title(sprintf('t=%0.1f',tt))        
end

