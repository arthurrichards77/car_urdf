function g = car_g(x,prob)

[v,~,~,vle0] = car_spds(x,prob);
[c_eq] = car_eqs(x,prob);
g = [v;
     c_eq;
     prob.A*x];
 
 % alternative form for <=0
 g = [vle0;
     c_eq;
     prob.A*x];