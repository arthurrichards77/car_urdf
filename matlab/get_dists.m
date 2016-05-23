function [ells,ts,vss,ss] = get_dists(dts,vs,prob)

taus = linspace(0,1,50);

ells = [];
ts = [];
vss = [];
ss = [];

% max speed
for ii=1:prob.n_cars,
    % stores for this car only
    this_ells= [];
    this_ts = [];
    this_vs = [];
    this_ss = [];
    % time and distance counters
    t_now = 0;
    ell_now = 0;
    for kk=1:prob.n_arcs,
        % wait and drive time
        dt_wait = dts(2*kk-1, ii);
        dt_arc = dts(2*kk, ii);
        % time vector
        this_ts = [this_ts; 
                   t_now + dt_wait + dt_arc*taus'];
        t_now = t_now + dt_wait + dt_arc;
        % velocities at start and end
        v0 = vs(kk,ii);
        v1 = vs(kk+1,ii);
        a = v0*dt_arc/prob.ell_arcs(kk,ii);
        b = v1*dt_arc/prob.ell_arcs(kk,ii);
        for tt = taus,
            % speed
            s_dot = a*(1-tt)*(1-tt) + b*tt*tt + (6-2*a-2*b)*tt*(1-tt);
            v = s_dot*prob.ell_arcs(kk,ii)/dt_arc;
            this_vs = [this_vs; v];
            % distance
            s = a*(1-tt)*(1-tt)*tt - b*tt*tt*(1-tt) + tt*tt*tt + 3*tt*tt*(1-tt);
            ell = ell_now + s*prob.ell_arcs(kk,ii);
            this_ells = [this_ells; ell];
            this_ss = [this_ss; s+(kk-1)];
        end
        ell_now = ell_now + prob.ell_arcs(kk,ii);            
    end
    vss = [vss this_vs];
    ells = [ells this_ells];
    ts = [ts this_ts];
    ss = [ss this_ss];
end