%% Obs(x,y) <=0 ---> Safe(x,y)>=0 ---> Trajectory safety condition: Safe(Px(t),Py(t))>=0 for all t0=<t=<tf
%%
function status=func_3D_SOS_Tube_yalmip(Safe,Px,Py,Pz,t0,tf,R,d)  
% Safe: Safety Constraints, Obs(x) <=0 ---> Safe(x)>=0 --->Safe(x(t),y(t),z(t))>=0 for all t0 =<t=< tf and (xt,yt) in {R^2-xt^2-yt^2-z^2} 
% (Px,Py,Pz): given polynomial trajectory
% [t0,tf]:start and final time, i.e., t in [t0 tf]
% R: radius of tube
% d: SOS relaxation order d>= order of given polynomial trajectory

% time
sdpvar t xt yt zt

[s1,c1] = polynomial(t,d);
[s2,c2] = polynomial(t,d);

% SOS condition: Safe(x(t)+xt,y(t)+yt,z(t)+zt)>=0 for all t0 =<t=< tf and (xt,yt) in {R^2-xt^2-yt^2-z^2} 
F = [sos(Safe(Px+xt,Py+yt,Pz+zt,t)-s1*(t-t0)*(tf-t)-s2*(R^2-xt^2-yt^2-zt^2) ), sos(s1), sos(s2)];

ops = sdpsettings('solver','mosek');
sol=solvesos(F,[],ops,[c1,c2]);


if sol.problem==0;
     status = 1
    display('Trajectory is safe.')
else
       status = 0
    display('Trajectory is NOT safe.') 
end


end