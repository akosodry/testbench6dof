function [sys,x0,str,ts] = system_dynamics(t,x,u,flag)
switch flag,
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes; 
  case 1
    sys = mdlDerivatives(t,x,u); 
  case 3
    sys = mdlOutputs(t,x,u); 
  case { 2, 4, 9 } 
    sys = [];
  otherwise
    error(['Unhandled flag = ',num2str(flag)]); 
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;     
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0 = [zeros(12,1)]';

str = [];
ts = [0 0];

function sys = mdlDerivatives(t,x,u)

g = evalin('base','g');
mb = evalin('base','mb');
mj1 = evalin('base','mj1');
mj2 = evalin('base','mj2');
mj3 = evalin('base','mj3');
Jb_phi = evalin('base','Jb_phi');
Jb_the = evalin('base','Jb_the');
Jb_psi = evalin('base','Jb_psi');
f1 = evalin('base','f1');
f2 = evalin('base','f2');
f3 = evalin('base','f3');
f4 = evalin('base','f4');
f5 = evalin('base','f5');
f6 = evalin('base','f6');

mj = [mj1;mj2;mj3];
Jb = [Jb_phi; Jb_the; Jb_psi];

M = diag([mj'+ones(1,3)*mb,Jb']);
V = [zeros(1,2), (mb+mj3)*g, zeros(1,3)]';

sys(1) = x(7);                      
sys(2) = x(8);                      
sys(3) = x(9);                      
sys(4) = x(10);                      
sys(5) = x(11);                      
sys(6) = x(12);                      

taua = [u(1);u(2);u(3);u(4);u(5);u(6)];
tauf = diag([f1,f2,f3,f4,f5,f6])*x(7:12);

ddx = M\(taua - tauf - V);

sys(7:12) = ddx;


function sys = mdlOutputs(t,x,u)
sys = x;