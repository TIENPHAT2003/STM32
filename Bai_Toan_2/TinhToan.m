syms m M l x theta theta_cham F x_cham_cham theta_cham_cham g
f1 = (m+M)*x_cham_cham + m*l*theta_cham_cham*cos(theta)-m*l*(theta_cham)^2*sin(theta)-F;
f2 = m*l*x_cham_cham*cos(theta)+m*l^2*theta_cham_cham-m*g*l*sin(theta);

[x_cham_cham, theta_cham_cham] = solve(f1,f2,x_cham_cham,theta_cham_cham) 