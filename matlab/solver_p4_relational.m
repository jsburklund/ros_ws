syms p2 p4 p6 p2_dot p4_dot p6_dot
syms l0 l4 l6
syms g

ratio = 0.5;  % Must be < 1 since p6 has max velocity of 4 rad/s and p4
            % has max velocity of 2 rad/s

h = l0 - (l4*cos(p4) + l6*cos(p4+p6));
h_dot = l4*sin(p4)*p4_dot + l6*sin(p4+p6)*(p4_dot+p6_dot);
omega = p4_dot + p6_dot;
thetai = p4 + p6;
eqn = (3*sym(pi)/2 - thetai) / omega == h_dot/g + sqrt((h_dot^2)/(g^2) + 2*h/g);

% Add in joint relationalvelocity constraints
eqn = subs(eqn, [p4 p4_dot], [p6*ratio p6_dot*ratio]);

solutions = solve(subs(eqn, [g l0 l4 l6], [9.81 1.2 0.374 0.229]), p6_dot)

% Plot a bunch of solutions
thetas = linspace(0,pi/2, 100);
results = zeros(1, size(thetas,2));
for i=1:size(thetas,2)
    results(i) = subs(solutions(1), [p6], [thetas(i)]);
end

figure();
plot(thetas, results);
xlabel('Phi (radians)'); ylabel('Phi dot (rad/sec)');
title('Release Angle vs. Required Joint Speed');
hold on
plot(thetas, results*ratio);
legend('Phi6 dot', 'Phi4 dot');