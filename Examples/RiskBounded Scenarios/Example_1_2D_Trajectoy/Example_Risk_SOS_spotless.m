%% SOS-based Continuous-Time Safety Verification in Spotless
clc; clear;close all, warning off

% time
t = msspoly('t',1); 

% trajectory (x(t)=Px, y(t)=Py)
Px=t; % trajectory x(t)
Py=((t-5)^4 + 2*(t-5)^3 - 15*(t-5)^2 - 12*(t-5) + 36)/20; % trajectory y(t)

% start and final time, i.e., t in [t0 tf]
t0=0;tf=9; 



% Risk Contours Delta=0.01, Being Safe: >=0 
% C * A_SOS_Cons_1>=0 (C is a alrge positive number to avoid numerical issues) and A_SOS_Cons_2>=0
safe_1= @(x1,x2,t) (1708377*x1)/43750 + (560140647*x2)/56000000 - (99*x1^2*x2^2)/200 - (4059*x1*x2)/500 + (x1^2 - 4*x1 + x2^2/4 - (41*x2)/40 + 12937/2625)^2 + (99*x1*x2^2)/50 + (4059*x1^2*x2)/2000 - (4480377*x1^2)/175000 + (198*x1^3)/25 - (2435829*x2^2)/700000 - (99*x1^4)/100 + (4059*x2^3)/8000 - (99*x2^4)/1600 - 134750939037/5600000000;%>= 0 
Safe_1= @(x1,x2,t) safe_1(x1,x2,t)*1000; % C * A_SOS_Cons_1>=0 where C=1000
Safe_2= @(x1,x2,t) x1^2 - 4*x1 + x2^2/4 - (41*x2)/40 + 12937/2625;



% SOS relaxation order
d=2;

% Safe(x)>=0 --->Safe(x(t),y(t))>=0 for all t0 =<t=< tf
status_1=func_2D_SOS_spotless(Safe_1,Px,Py,t0,tf,d) ;
status_2=func_2D_SOS_spotless(Safe_2,Px,Py,t0,tf,d) ;
clc; 
if status_1 ==1 & status_2 ==1
    status =1;   display('Trajectory is safe.')
else
    status =0;   display('Trajectory is NOT safe.')
end

%% visualization
% obs/safe
hold off; syms x1 x2;
fcontour(Safe_1(x1,x2,0),'LevelList',[0 0],'LineWidth',1,'LineColor','r');hold on
fcontour(Safe_2(x1,x2,0),'LevelList',[0 0],'LineWidth',1,'LineColor','b');hold on
% trajectory
PPx=[];PPy=[];
for tt=[t0:0.1:tf];  PPx=[PPx,double(subs(Px,t,tt))]; PPy=[PPy,double(subs(Py,t,tt))];end
plot(PPx(1),PPy(1),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
plot(PPx(end),PPy(end),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
plot(PPx,PPy,'--','LineWidth',2);grid on; 

