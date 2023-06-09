clear; clc;

syms ax hx gx ay hy gy h g

eqn=h^2==(ax-sqrt(g^2-gy^2))^2+(ay-gy)^2;

sol=solve(eqn,gy);
sol=simplify(sol);

sol=[(ax*((2*g*h)*(2*g*h))^(1/2) + ax^2*ay + ay*g^2 - ay*h^2 + ay^3)/(2*(ax^2 + ay^2));(ax^2*ay - ax*((2*g*h)*(2*g*h))^(1/2) + ay*g^2 - ay*h^2 + ay^3)/(2*(ax^2 + ay^2))]

gy=inline(sol);
