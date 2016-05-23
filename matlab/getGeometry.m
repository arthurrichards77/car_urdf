function prob = getGeometry(prob)

% default lengths are just straight line distances
prob.ell_arcs = sqrt(diff(prob.arc_xs).^2 + diff(prob.arc_ys).^2);

% number of points for plotting
prob.n_arc_p = 50;

% get detailed model of each arc
for cc=1:prob.n_cars,
    for ii=1:prob.n_arcs,
        % indices for storing
        inds = (ii-1)*prob.n_arc_p + (1:prob.n_arc_p);
        % start and end points
        xs = prob.arc_xs(ii,cc);
        ys = prob.arc_ys(ii,cc);
        xf = prob.arc_xs(ii+1,cc);
        yf = prob.arc_ys(ii+1,cc);
        hs = prob.arc_hdgs(ii,cc);
        hf = prob.arc_hdgs(ii+1,cc);
        % baseline direction of arc, i.e. between endpoints
        theta = atan2(yf-ys,xf-xs);
        % heading offsets from baseline at either end
        dhs = hs - theta;
        dhf = hf - theta;
        % linear distance for interp
        prob.arc_sp(inds,cc) = (ii-1)+linspace(0,1,prob.n_arc_p);
        % heading always linear in both cases
        prob.arc_hp(inds,cc) = linspace(hs,hf,prob.n_arc_p);
        if (dhs==0)&&(dhf==0),
            % straight line case
            prob.arc_xp(inds,cc) = linspace(xs,xf,prob.n_arc_p);
            prob.arc_yp(inds,cc) = linspace(ys,yf,prob.n_arc_p);
        elseif abs(dhs+dhf)<1e-6,
            % circular arc case
            % radius of turn
            R = -0.5*prob.ell_arcs(ii,cc)/sin(dhs);
            % centre of turn
            xc = xs - R*sin(hs);
            yc = ys + R*cos(hs);
            % reconstruct segment
            prob.arc_xp(inds,cc) = xc + R*sin(prob.arc_hp(inds,cc));
            prob.arc_yp(inds,cc) = yc - R*cos(prob.arc_hp(inds,cc));
            % overwrite arc length
            prob.ell_arcs(ii,cc) = abs(R*(hf-hs));
            % and reduce max speed if appropriate
            prob.v_max(ii,cc) = min([prob.v_max(ii,cc) sqrt(abs(prob.lat_acc*R))]);
        else
            dhs
            dhf
            dhs+dhf
            error('Neither line nor arc');
        end
    end
end

