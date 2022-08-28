function Output = M_Full (this, P, Beta)
d1 = 0.5;
d2 = 0.5;
g = 9.8;

Output = [ ...
[Beta(1) + Beta(2) + 2*d1*(Beta(3)*cos(P(2)) - Beta(4)*sin(P(2))), Beta(2) + d1*Beta(3)*cos(P(2)) - d1*Beta(4)*sin(P(2))] 	
[Beta(2) + d1*Beta(3)*cos(P(2)) - d1*Beta(4)*sin(P(2)), Beta(2) + Beta(5)] 	
];

end