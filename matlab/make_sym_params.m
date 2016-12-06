function sym_params = make_sym_params(params)

% default
sym_params = struct('dummy',[]);

% extract field names
param_names = fieldnames(params);
num_params = numel(param_names);

for ii=1:num_params,
    sym_params.(param_names{ii})=sym(param_names{ii},size(getfield(params,param_names{ii})));
end

% remove dummy field if never used
if numel(sym_params.dummy)==0,
    sym_params = rmfield(sym_params,'dummy');
end