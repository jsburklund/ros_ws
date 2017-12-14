syms p2 p4 p6 p2_dot p4_dot p6_dot
syms l0 l4 l6
syms g

h = l0 - (l4*cos(p6) + l6*cos(2*p6));
h_dot = l4*sin(p6)*p6_dot + 2*l6*sin(2*p6)*p6_dot;
eqn = (3*sym(pi)/2 + 2*p6) / (2*p6_dot) == h_dot/g + sqrt((h_dot^2)/(g^2) + 2*h/g);

solutions = solve(subs(eqn, [g l0 l4 l6], [9.81 1.2 0.374 0.229]), p6_dot)

% Plot a bunch of solutions
thetas = linspace(0,pi/2, 100);
results = zeros(1, size(thetas,2));
for i=1:size(thetas,2)
    results(i) = subs(solutions(2), [p6], [thetas(i)]);
end

figure();
plot(thetas, results); xlabel('Phi 6'); ylabel('Phi 6 dot'); title('Release Angle vs. Required Joint Speed');