%% Obs(x,y) <=0 ---> Safe(x,y)>=0 ---> Trajectory safety condition: Safe(Px(t),Py(t))>=0 for all t0=<t=<tf
%%
function status=func_3D_SOS_Tube_spotless(Safe,Px,Py,Pz,t0,tf,R,d)

% Safe: Safety Constraints, Obs(x) <=0 ---> Safe(x)>=0 --->Safe(x(t),y(t),z(t))>=0 for all t0 =<t=< tf
% (Px,Py,Pz): given polynomial trajectory
% [t0,tf]:start and final time, i.e., t in [t0 tf]
% R: radius of tube
% d: SOS relaxation order d>= order of given polynomial trajectory


t = msspoly('t',1);
xt = msspoly('xt',1);
yt = msspoly('yt',1);
zt = msspoly('zt',1);

prog = spotsosprog;
prog = prog.withIndeterminate( [t,xt,yt,zt] );
[prog,gamma] = prog.newFree(1);

bases = monomials( [t,xt,yt,zt], 0:d );

[ prog, s1 ] = prog.newFreePoly( bases ); 
prog = prog.withSOS(s1);

[ prog, s2 ] = prog.newFreePoly( bases ); 
prog = prog.withSOS(s2);

% SOS condition: Safe(x(t)+xt,y(t)+yt,z(t)+zt)>=0 for all t0 =<t=< tf and (xt,yt) in {R^2-xt^2-yt^2-zt^2} 
prog = prog.withSOS( Safe(Px+xt,Py+yt,Pz+zt,t)-gamma -s1*(t-t0)*(tf-t)-s2*(R^2-xt^2-yt^2-zt^2) ); 

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