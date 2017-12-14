%% Arm Parameters
syms th1 th2 th3 th4 th5 th6
l0 = 270.35; l1=69.0; l2=364.35; l3=69.0; l4=374.29; l5=10; l6=368.30;

%% Generate transforms for a 2D arm graph.
A1 = dh2transform(0, l0, l1, -pi/2);  % Ground to first joint
A2 = dh2transform(th2+pi/2, 0, 0, pi/2) * dh2transform(0, l2, l3, -pi/2);
A3 = dh2transform(th4, 0, 0, pi/2) * dh2transform(0, l4, l5, -pi/2);
A4 = dh2transform(th6, 0, 0, pi/2) * dh2transform(0, 280, 0, 0);  % to tip frame

%% Test drawing the arm;
% Get the arm points
pts = [[0; 0; 0; 1] ...
       subs(A1,          [th2, th4, th6], [0 0 0])*[0; 0; 0; 1] ...
       subs(A1*A2,       [th2, th4, th6], [0 0 0])*[0; 0; 0; 1] ...
       subs(A1*A2*A3,    [th2, th4, th6], [0 0 0])*[0; 0; 0; 1] ...
       subs(A1*A2*A3*A4, [th2, th4, th6], [0 0 0])*[0; 0; 0; 1] ];
   
% Plot the points
figure();
plot(pts(1,:), pts(3,:));
axis equal;

% Plot a moving arm
figure();
pts = zeros(4,90);
for i=1:90
    pts(:,i) = subs(A1*A2*A3*A4, [th2, th4, th6], [0 i*pi/180 0])*[0; 0; 0; 1];
end
scatter(pts(1,:), pts(3,:));
axis equal;

%% Generate a trajectory between start and end points
syms p4_start p6_start p4_end p6_end p4_dot_start p6_dot_start p4_dot_end p6_dot_end
syms a_4 a_6 t

% Start time = 0, calculate end  time based on linear ramp of joint angles
eqn = p6_end-p6_start == 1/2*(p6_dot_end-p6_dot_start)*t;
t_soln = solve(subs(eqn, [p6_start p6_end p6_dot_start p6_dot_end], [0 0.9996 0 3.554]), t);

t_soln = t_soln * 1;

% 4 knowns: polynomial degree 3
% q(t) = a1 + a2t + a3t^2 + a4t^3
% v(t) = a2 + 2a3t + 3a4t^2
b = [p6_start; p6_dot_start; p6_end; p6_dot_end;];
b = subs(b, [p6_start p6_end p6_dot_start p6_dot_end], [0 0.9996 0 3.554]);

M = [1  0      0        0; ...
     0  1      0        0; ...
     1  t_soln t_soln^2 t_soln^3; ...
     0  1      2*t_soln 3*t_soln^2; ];
 
A6 = inv(M)*b;

% Plot the results in joint space
ts = linspace(0, t_soln, 100);
pos = zeros(1, size(ts,2));
vel = zeros(1, size(ts,2));
for i=1:size(ts,2)
    pos(i) = A6(1) + A6(2)*ts(i) + A6(3)*(ts(i)^2) + A6(4)*(ts(i)^3);
    vel(i) = A6(2) + 2*A6(3)*ts(i) + 3*A6(4)*(ts(i)^2);
end
figure();
plot(ts, pos); title('Positions'); hold on;
plot(ts, pos/2);
figure();
plot(ts, vel); title('Velocities'); hold on;
plot(ts, vel/2);
