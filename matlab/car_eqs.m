function [c_eq] = car_eqs(x,prob)

[dts,vs] = get_vars(x,prob);

c_eq=[];

% continuity at arc ends
for ii=1:prob.n_cars,
    for kk=1:prob.n_arcs,
        dt_wait = dts(2*kk-1, ii);
        v0 = vs(kk,ii);
        c_eq = [c_eq; v0*dt_wait];
    end
end            