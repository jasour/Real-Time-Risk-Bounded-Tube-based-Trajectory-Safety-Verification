%% Obs(x,y) <=0 ---> Safe(x,y)>=0 ---> Trajectory safety condition: Safe(Px(t),Py(t))>=0 for all t0=<t=<tf
%%
function status=func_2D_SOS_spotless(Safe,Px,Py,t0,tf,d)

% Safe: Safety Constraints, Obs(x) <=0 ---> Safe(x)>=0 --->Safe(x(t),y(t))>=0 for all t0 =<t=< tf
% (Px,Py): given polynomial trajectory
% [t0,tf]:start and final time, i.e., t in [t0 tf]
% d: SOS relaxation order d>= order of given polynomial trajectory


t = msspoly('t',1); % time

prog = spotsosprog;
prog = prog.withIndeterminate( t );

[prog,gamma] = prog.newFree(1);

bases = monomials( t, 0:d );

[ prog, s1 ] = prog.newFreePoly( bases ); 
prog = prog.withSOS(s1);

% SOS condition: Safe(x(t),y(t))>=0 for all t0 =<t=< tf
prog = prog.withSOS( Safe(Px,Py,t)-gamma-s1*(t-t0)*(tf-t));  

spot_options = spot_sdp_default_options();
spot_options.verbose = 0; % printing information
sol = prog.minimize(-gamma, @spot_mosek,spot_options);


if double(sol.eval(gamma)) >=0 
    status = 1
    display('Trajectory is safe.')
else
    status = 0
    display('Trajectory is NOT safe.') 
end
end