function Output = W_Full (this, P, V, A)
d1 = 0.5;
d2 = 0.5;
g = 9.8;

Output = [ ...
[A(1), A(1) + A(2), d1*V(1)^2*sin(P(2)) - d1*(sin(P(2))*(V(1) + V(2))^2 - cos(P(2))*(A(1) + A(2))) + d1*A(1)*cos(P(2)), d1*V(1)^2*cos(P(2)) - d1*(sin(P(2))*(A(1) + A(2)) + cos(P(2))*(V(1) + V(2))^2) - d1*A(1)*sin(P(2)), 0, V(1), 0, sign(V(1)), 0] 	
[0, A(1) + A(2), d1*V(1)^2*sin(P(2)) + d1*A(1)*cos(P(2)), d1*V(1)^2*cos(P(2)) - d1*A(1)*sin(P(2)), A(2), 0, V(2), 0, sign(V(2))] 	
];

end