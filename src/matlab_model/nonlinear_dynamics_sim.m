% This script simulates the dynamics of the robot using a non-linear state 
% space model. This script is primarilty used to validate the non-linear
% state space model.

clear;clear all;clc;

syms x y z roll_ pitch yaw u v w p q r du0 du1 du2 du3 du4 du5 radius

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set initial state
state_0 = [0 0 0 0 0 0 0.5 0 0 0 0 0];

% set target state
% Note: All elements MUST be set to zero
target_state = [0 0 0 0 0 0 0 0 0 0 0 0];

% Control input
du = [0 0 0 0 0 0];

% Set simulation parameters
t_lower = 0;
t_upper = 30;
h = 0.1;
t_span = t_lower:h:t_upper;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set up symbolic linear state space model
[state_symbolic,du_symbolic,F_dot,f2] = symbolic_state_space();

% upload parameters (all SI units)
mass = 27.0;
Ix = 1.0;
Iy = 0.9;
Iz = 1.38;
Ixy = 0.0;
Ixz = 0.0;
Iyz = 0.0;
mzg = 0.3;
Xu_dot = 35.0;
Yv_dot = 55.0;
Zw_dot = 45.0;
Kp_dot = 1.2;
Mq_dot = 1.2;
Nr_dot = 1.5;
Xq_dot = 0.5;
Yp_dot = 0.5;
Xu = 25.0;
Xuu = 50.0;
Yv = 35.0;
Yvv = 60.0;
Zw = 45.0;
Zww = 70;
Kp = 15.0;
Kpp = 30.0;
Mq = 15.0;
Mqq = 30.0;
Nr = 15.0;
Nrr = 30.0;
gx = 0.0; 
gy = 0.0; 
gz = 0.0;
bx = 0.0;
by = 0.0;
bz = 0.0;
gravity = 9.81; 
vehicle_radius = 0.188; 
water_density = 1000.0; 
ctf = 0.00006835*550;
ctb = 0.8*ctf;
gravity_effects = 0;
Q = 0;
R = 0;

% Substitute constant parameters
state_dot = subs(F_dot);

% Create Matlab function of state space model
nonlinear_state_sym(x, y, z, roll_, pitch, yaw, u, v, w, p, q, r, radius, du0, du1, du2, du3, du4, du5) = state_dot;
nonlinear_state = matlabFunction(nonlinear_state_sym);
df_dcontrol = 0;

% Simulate state space
[t_span,state] = ode45(@(t,state)nonlinear_sim(state,nonlinear_state,du,vehicle_radius),t_span,state_0);

% Plot simulation results
clf
figure(1)
subplot(2,1,1)
plot(t_span,state(:,1:6))
title('Auri Pose')
xlabel('time (s)')
ylabel('(m) or (rad)')
legend('x','y','z','roll','pitch','yaw')

hold on

subplot(2,1,2)
plot(t_span,state(:,7:12))
title('Auri Velocity')
xlabel('time (s)')
ylabel('(m/s) or (rad/s)')
legend('u','v','w','p','q','r')

function [ret] = nonlinear_sim(state,nonlinear_state,du,vehicle_radius)
% evaluates the state via ode45. this is where lqr is implemented.

    % set state
    x = state(1);
    y = state(2);
    z = state(3);
    roll_ = state(4);
    pitch = state(5);
    yaw = state(6);
    u = state(7);
    v = state(8);
    w = state(9);
    p = state(10);
    q = state(11);
    r = state(12);

    % set control input
    du0 = du(1);
    du1 = du(2);
    du2 = du(3);
    du3 = du(4);
    du4 = du(5);
    du5 = du(6);

    % If the vehicle moves out of the water the flotability decreases
    if z < 0.0
        radius = vehicle_radius - abs(z);
        if radius < 0.0
            radius = 0.0;
        end
    else 
        radius = vehicle_radius; 
    end
    
    ret = double(nonlinear_state(x,y,z,roll_,pitch,yaw,u,v,w,p,q,r,radius,du0,du1,du2,du3,du4,du5));
end