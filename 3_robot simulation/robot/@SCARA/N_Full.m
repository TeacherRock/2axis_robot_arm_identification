function Output = N_Full (this, P, V, Beta)
d1 = 0.5;
d2 = 0.5;
g = 9.8;

Output = [ ...
Beta(8)*sign(V(1)) - d1*(Beta(4)*cos(P(2))*(V(1) + V(2))^2 + Beta(3)*sin(P(2))*(V(1) + V(2))^2) + Beta(6)*V(1) + d1*Beta(4)*V(1)^2*cos(P(2)) + d1*Beta(3)*V(1)^2*sin(P(2)) 	
Beta(9)*sign(V(2)) + Beta(7)*V(2) + d1*Beta(4)*V(1)^2*cos(P(2)) + d1*Beta(3)*V(1)^2*sin(P(2)) 	
];

end