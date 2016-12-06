function [v,vhi,vlo,vle0] = car_spds(x,prob)

[dts,vs] = get_vars(x,prob);
ts = linspace(0,1,2+prob.num_spd_pts);
ts = ts(2:(1+prob.num_spd_pts));
v=[];
vle0=[]; % additional constraint form to avoid division v*t-vmax*t<=0; -v*t<=0;
vhi = [];
vlo = [];

for ii=1:prob.n_cars,
    for kk=1:prob.n_arcs,
        dt_arc = dts(2*kk, ii);
        v0 = vs(kk,ii);
        v1 = vs(kk+1,ii);
        for tt = ts,
            vn = v0*(1-tt)*(1-tt) + v1*tt*tt + (6*prob.ell_arcs(kk,ii)/dt_arc-2*v0-2*v1)*tt*(1-tt);
            v = [v; vn];
            vhi = [vhi; prob.v_max(kk,ii)]; 
            vlo = [vlo; 0];
            vdt = v0*dt_arc*(1-tt)*(1-tt) + v1*dt_arc*tt*tt + (6*prob.ell_arcs(kk,ii)-2*v0*dt_arc-2*v1*dt_arc)*tt*(1-tt);
            vle0 = [vle0; vdt-prob.v_max(kk,ii)*dt_arc; -vdt];            
        end
        
    end
end