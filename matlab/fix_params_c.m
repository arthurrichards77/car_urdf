function s2 = fix_params_c(s1,params)

% default
s2 = s1;

% extract field names
param_names = fieldnames(params);
num_params = numel(param_names);

for ii=1:num_params,
    p = getfield(params,param_names{ii});
    if ismatrix(p),
        s2 = fix_matrix_c(s2,param_names{ii},size(p,1),size(p,2));
    elseif isvector(p),
        s2 = fix_vector_c(s2,param_names{ii},size(p,1),size(p,2));
    else
        error('One of the parameters is neither a vector nor a matrix')
    end
end