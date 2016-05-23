function [c,c_eq] = car_cons(x,prob)

[dts,vs] = get_vars(x,prob);

c_eq=[];
c=[];

ts = linspace(0,1,7);

% max speed
for ii=1:prob.n_cars,
    for kk=1:prob.n_arcs,
        dt_arc = dts(2*kk, ii);
        v0 = vs(kk,ii);
        v1 = vs(kk+1,ii);
        a = v0*dt_arc/prob.ell_arcs(kk,ii);
        b = v1*dt_arc/prob.ell_arcs(kk,ii);
        for tt = ts,
            s_dot = a*(1-tt)*(1-tt) + b*tt*tt + (6-2*a-2*b)*tt*(1-tt);
            v = s_dot*prob.ell_arcs(kk,ii)/dt_arc;
            c = [c; v-prob.v_max(kk,ii); -v];
        end
    end
end

% continuity at arc ends
for ii=1:prob.n_cars,
    for kk=1:prob.n_arcs,
        dt_wait = dts(2*kk-1, ii);
        v0 = vs(kk,ii);
        c_eq = [c_eq; v0*dt_wait];
    end
end            