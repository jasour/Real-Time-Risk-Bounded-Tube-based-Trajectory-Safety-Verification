% This code computes risk bounded safe sets (Risk Contours) in the presence of uncertain safety constraints
% Ashkan Jasour, Weiqiao Han, Brian Williams,"Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments", Robotics: Science and Systems (RSS), 2021.
% Paper: http://www.roboticsproceedings.org/rss17/p069.pdf

%%
clc; clear all; close all
%% position: [x1 x2 x3], uncertain parameter: w1, w2, w3, w4 *******
syms x1 x2 x3 w1 w2 w3 w4
Delta=0.01; % Acceptable risk level
%% uncertain object g={(x1,x2): g(x1,x2,x3 w1,w2,w3,w4)>=0 } with uncertain parameter w1, w2, w3, w4 *****
% g : ellipse with uncertain size and location, uncertain parameters: w1,w2, w3, w4
% w1: uniform distribution on [l,u]
% w2 has normal distribution on [mean,var],   -inf<= w2 <=inf 
% w3 has beta distribution [a,b],  0<= w3 <=1
% w4 has normal distribution on [mean,var],   -inf<= w4 <=inf
g = (w1)^2-((x1-2-0.1*w2)/1)^2-((x2-2-0.1*w3)/2)^2 -((x3-2-0.1*w4)/2)^2;

%%
% degree of uncertain object g 
dg=polynomialDegree(g);
%% statistics (moments) of probability distributions of uncertain parameters  ******
% see section II of paper

% w1 has Uniform distribution on [l,u],  l<= w3 <=u
u=0.4;l=0.3; 
% moments sequence of w1
m_w1=[1];for i=1:2*dg ;m_w1(i+1,1)=(1/(u-l))*((u^(i+1) - l^(i+1))/(i+1));end

% w2 has normal distribution on [mean,var],   -inf<= w2 <=inf 
mean=0; var=0.1;    
% moments sequence of w2
for k=0:2*dg; m_w2(k+1,1)=sqrt(var)^k*(-j*sqrt(2))^k*kummerU(-k/2, 1/2,-1/2*mean^2/var);end

% w3 has beta distribution [a,b],  0<= w3 <=1
a=3;b=3; 
% moments sequence of w3
m_w3=[1];for k=1:2*dg; m_w3=[m_w3;(a+k-1)/(a+b+k-1)*m_w3(end) ]; end;

% w2 has normal distribution on [mean,var],   -inf<= w2 <=inf 
mean=0; var=0.1;    
% moments sequence of w4
for k=0:2*dg; m_w4(k+1,1)=sqrt(var)^k*(-j*sqrt(2))^k*kummerU(-k/2, 1/2,-1/2*mean^2/var);end


%% Computes the risk-bounded safe locations (Risk Contours) in the presence of uncertain moving object 
% Eq (9),(10), and Theorem I of the paper

% Calculate the first and Second order moments of uncertain object
Mg=[]; %List of first and second order moments
for dd=1:2
% Moment of order dd
Md=expand(g^dd);
% Replace moments of uncertain parameter w1
Md1=subs(Md,flip(w1.^[1:dd*dg].'),flip(m_w1(2:dd*dg+1))) ; 
% Replace moments of uncertain parameter w2
Md2=subs(Md1,flip(w2.^[1:dd*dg].'),flip(m_w2(2:dd*dg+1))) ; 
% Replace moments of uncertain parameter w3
Md3=subs(Md2,flip(w3.^[1:dd*dg].'),flip(m_w3(2:dd*dg+1))) ; 
% Replace moments of uncertain parameter w4
Md4=subs(Md3,flip(w4.^[1:dd*dg].'),flip(m_w4(2:dd*dg+1))) ; 
Mg=[Mg;Md4];
end

%% A) Risk-bounded safe locations (Risk Contours) in the presence of uncertain moving object whose risk <= Delta 
% safe set= {(x1,x2):  Cons_1 <= Delta , Cons_2 <=0} ( Eq(12) in the paper)
% *********************
% Paper: Ashkan Jasour, Weiqiao Han, Brian Williams,"Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments", Robotics: Science and Systems (RSS), 2021.
% http://www.roboticsproceedings.org/rss17/p069.pdf

% A_Cons_1=(Mg(2)-Mg(1)^2)/Mg(2);   % <= Delta
A_Cons_1= (1-Delta)*Mg(2)-Mg(1)^2;   %  <=0 
A_Cons_2=Mg(1); % This constraint should be <=0  since being safe is <=0  (according to safety constraints)

% SOS (positivity) Constraints
A_SOS_Cons_1= -A_Cons_1;  %  >=0 
A_SOS_Cons_2= -A_Cons_2; % >=0
%% B) Risk-bounded safe locations (Risk Contours) in the presence of uncertain moving object whose risk <= Delta
% safe set= {(x1,x2):  Cons_1 <= Delta , Cons_2 >=0, Cons_3 >= 0 } ( Eq(6) in the paper)
% *********************
% Paper: Weiqiao Han, Ashkan Jasour, Brian Williams,"Non-Gaussian Risk Bounded Trajectory Optimization for Stochastic Nonlinear Systems in Uncertain Environments", 39th IEEE Conference on Robotics and Automation (ICRA), 2022.
% https://arxiv.org/pdf/2203.03038.pdf

%B_Cons_1=(4/9)*(Mg(2)-Mg(1)^2)/Mg(2);  % <= Delta
B_Cons_1=(4/9-Delta)*Mg(2)-(4/9)*Mg(1)^2;  % <= 0
B_Cons_2 = Mg(1)^2-5/8*Mg(2);  % >= 0 
B_Cons_3=Mg(1); % This constraint should be <=0  since being safe is <=0  according to the defined safety constraints. In the papre, being safe is >=0; Hence, Cons_3 >=0.

% SOS (positivity) Constraints 
B_SOS_Cons_1 = -B_Cons_1; % >=0
B_SOS_Cons_2 = B_Cons_2; % >=0
B_SOS_Cons_3 = -B_Cons_3 ; % >=0



%% Plots
clc;display('Done!'); display('Working on plots')
%% Risk contour: outside of the outer curve, risk of collision with the uncertain object is less or equal to Delta = 0.1.
[x1,x2, x3]=meshgrid([-10:0.1:10],[-10:0.1:10], [-10:0.1:10]);


%% Plot A
figure
p=patch(isosurface(x1,x2,x3,eval(A_SOS_Cons_1),0));
set(p,'FaceColor','red','EdgeColor','none','FaceAlpha',0.3);hold on

p=patch(isosurface(x1,x2,x3,eval(A_SOS_Cons_2),0));
set(p,'FaceColor','red','EdgeColor','none','FaceAlpha',0.3);hold on

camlight; lighting gouraud
hold on;grid on;set(gca,'fontsize',31)
title({'Approach A: Risk-bounded safe locations(outside of the outer curve)'})



%% Plot B
figure
p=patch(isosurface(x1,x2,x3,eval(B_SOS_Cons_1),0));
set(p,'FaceColor','red','EdgeColor','none','FaceAlpha',0.3);hold on

p=patch(isosurface(x1,x2,x3,eval(B_SOS_Cons_2),0));
set(p,'FaceColor','red','EdgeColor','none','FaceAlpha',0.3);hold on

p=patch(isosurface(x1,x2,x3,eval(B_SOS_Cons_3),0));
set(p,'FaceColor','red','EdgeColor','none','FaceAlpha',0.3);hold on

camlight; lighting gouraud
hold on;grid on;set(gca,'fontsize',31)
title({'Approach B: Risk-bounded safe locations(outside of the outer curve)'})

