function [c] = car_ineqs(x,prob)

[v,vhi,vlo] = car_spds(x,prob);

c = [v - vhi; vlo - v];
