function g = car_g(x,prob)

[v,~,~] = car_spds(x,prob);
[c_eq] = car_eqs(x,prob);
g = [v;
     c_eq;
     prob.A*x];