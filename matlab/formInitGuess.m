function prob = formInitGuess(prob)

prob.dts0 = zeros(2*prob.n_arcs,prob.n_cars);
prob.dts0(2:2:end,:) = 1.5*prob.ell_arcs./prob.v_max;
prob.vs0 = ones(prob.n_arcs-1,prob.n_cars)*min(min(prob.v_max));

prob.dts_lb = 0*prob.dts0;
prob.dts_ub = prob.dts0 + inf;

prob.vs_lb = 0*prob.vs0;
prob.vs_ub = min(prob.v_max(2:end,:),prob.v_max(1:(end-1),:));

% reshape them to vector form
prob.x0 = reshape([prob.dts0; prob.vs0],(2*prob.n_arcs + (prob.n_arcs-1))*prob.n_cars,1);
prob.x_lb = reshape([prob.dts_lb; prob.vs_lb],(2*prob.n_arcs + (prob.n_arcs-1))*prob.n_cars,1);
prob.x_ub = reshape([prob.dts_ub; prob.vs_ub],(2*prob.n_arcs + (prob.n_arcs-1))*prob.n_cars,1);

