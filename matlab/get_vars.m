function [dts,vs] = get_vars(x,prob)

nvars_per_car = 2*prob.n_arcs + (prob.n_arcs-1);

dtvs = reshape(x,nvars_per_car,prob.n_cars);

dts = dtvs(1:(2*prob.n_arcs),:);
vs = dtvs((2*prob.n_arcs)+(1:(prob.n_arcs-1)),:);
vs = [prob.v_init; vs; 0*prob.v_init];