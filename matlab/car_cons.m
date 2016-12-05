function [c,c_eq] = car_cons(x,prob)

c = car_ineqs(x,prob);
c_eq = car_eqs(x,prob);