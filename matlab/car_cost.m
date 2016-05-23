function [J,g] = car_cost(x,prob)

%[dts,vs] = get_vars(x,prob);

%J = sum(sum(dts));
%J = -sum(sum(vs));

w_drive = 0.3;

%J = w_drive*sum(sum(dts(2:2:end,:))) + sum(sum(dts));

g = repmat([ones(2*prob.n_arcs,1); zeros(prob.n_arcs-1,1)],prob.n_cars,1) ...
  + w_drive*repmat([kron(ones(prob.n_arcs,1),[0;1]); zeros(prob.n_arcs-1,1)],prob.n_cars,1);

% test gradient
%test = [J g'*x]

J = g'*x;

