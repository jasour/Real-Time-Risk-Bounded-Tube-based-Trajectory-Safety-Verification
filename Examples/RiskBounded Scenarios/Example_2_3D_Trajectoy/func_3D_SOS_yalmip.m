%% Obs(x,y) <=0 ---> Safe(x,y)>=0 ---> Trajectory safety condition: Safe(Px(t),Py(t))>=0 for all t0=<t=<tf
%%
function status=func_3D_SOS_yalmip(Safe,Px,Py,Pz, t0,tf,d)
% Safe: Safety Constraints, Obs(x) <=0 ---> Safe(x)>=0 --->Safe(x(t),y(t),z(t))>=0 for all t0 =<t=< tf
% (Px,Py,Pz): given polynomial trajectory
% [t0,tf]:start and final time, i.e., t in [t0 tf]
% d: SOS relaxation order d>= order of given polynomial trajectory


% time
sdpvar t


[s1,c1] = polynomial(t,d);

% SOS condition: Safe(x(t),y(t))>=0 for all t0 =<t=< tf
F = [sos(Safe(Px,Py,Pz,t)-s1*(t-t0)*(tf-t)), sos(s1)];

ops = sdpsettings('solver','mosek');
sol=solvesos(F,[],ops,[c1]);

if sol.problem==0;
     status = 1
    display('Trajectory is safe.')
else
       status = 0
    display('Trajectory is NOT safe.') 
end


end