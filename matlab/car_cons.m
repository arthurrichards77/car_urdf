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
        for tt = ts,
            v = v0*(1-tt)*(1-tt) + v1*tt*tt + (6*prob.ell_arcs(kk,ii)/dt_arc-2*v0-2*v1)*tt*(1-tt);            
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