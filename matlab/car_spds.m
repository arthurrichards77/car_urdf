function [v,vhi,vlo] = car_spds(x,prob)

[dts,vs] = get_vars(x,prob);
ts = linspace(0,1,2+prob.num_spd_pts);
ts = ts(2:(1+prob.num_spd_pts));
v=[];
vhi = [];
vlo = [];

for ii=1:prob.n_cars,
    for kk=1:prob.n_arcs,
        dt_arc = dts(2*kk, ii);
        v0 = vs(kk,ii);
        v1 = vs(kk+1,ii);
        for tt = ts,
            vt = v0*(1-tt)*(1-tt) + v1*tt*tt + (6*prob.ell_arcs(kk,ii)/dt_arc-2*v0-2*v1)*tt*(1-tt);
            v = [v; vt];
            vhi = [vhi; prob.v_max(kk,ii)]; 
            vlo = [vlo; 0];
        end
        
    end
end