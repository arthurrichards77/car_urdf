function ipoptgen(obj,cons,x0,x_L,x_H,g_L,g_H,params)

%% symbolic decision variables
x = sym('x',[numel(x0) 1]);

% and parameters
sym_params = make_sym_params(params);

% objective and constraints
f = obj(x,sym_params);
g = cons(x,sym_params);

%% calculations of derivatives etc

% sizes
m = numel(g);
n = numel(x);

% now process derivatives

% objective gradient
grad_f = gradient(f,x);

% jacobian of constraints
jac_g = jacobian(g,x);
[iRow,jCol] = find(jac_g);

nnz_jac_g = numel(iRow);

for kk = 1:nnz_jac_g,
    jac_g_values(kk) = jac_g(iRow(kk),jCol(kk));
end
%jac_g_values

% hessian of lagrangian

% first need symbols for new multipliers
lambda = sym('lambda',size(g));
obj_factor = sym('obj_factor');

% now calculate the thing
hess = obj_factor * hessian(f,x);
for kk=1:m,
    hess = hess + lambda(kk)*hessian(g(kk),x);
end

% only need the bottom half
for ii=1:n,
    for jj = (ii+1):n,
        hess(ii,jj) = 0;
    end
end

% show it
%hess

% and to crazy spare format
[iRow_h,jCol_h] = find(hess);

nnz_h_lag = numel(iRow_h);

for kk = 1:nnz_h_lag,
    h_lag_values(kk) = hess(iRow_h(kk),jCol_h(kk));        
end
%h_lag_values

%% write parameter c

% start writing the file
fid = fopen('ipoptgen/mynlp_params_init.cpp','w');

param_names = fieldnames(params);

for kk = 1:numel(param_names),
    p = getfield(params,param_names{kk});
    if ismatrix(p),
        for ii=1:size(p,1),
            for jj=1:size(p,2),
                fprintf(fid,'%s[%d][%d] = %f;\n',param_names{kk},ii-1,jj-1,p(ii,jj));
            end
        end
    else
        error('One of the parameters is neither a vector nor a matrix')
    end   
end

fclose(fid);

%% write parameter header c

% start writing the file
fid = fopen('ipoptgen/mynlp_params.hpp','w');

% initial guess is always a parameter
fprintf(fid,'Number x_init[%d];\n',n);

% work through parameters
param_names = fieldnames(params);

for ii = 1:numel(param_names),
    p = getfield(params,param_names{ii});
    if ismatrix(p),
        fprintf(fid,'Number %s[%d][%d];\n',param_names{ii},size(p,1),size(p,2));
    elseif isvector(p),
        fprintf(fid,'Number %s[%d] = %s;\n',param_names{ii},numel(p));
    else
        error('One of the parameters is neither a vector nor a matrix')
    end   
end

fclose(fid);

%% write C code

% NLP class name
nlp_class_name = 'HS071_NLP';

% start writing the file
fid = fopen('ipoptgen/mynlp.cpp','w');

fprintf(fid,['#include "mynlp.hpp"\n' ...
             '#include <cassert>\n'...
             '#include <cmath>\n'...
             'using namespace Ipopt;\n\n']);

% NLP info method
fprintf(fid,'bool %s::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, IndexStyleEnum& index_style)\n{\n',nlp_class_name);
fprintf(fid,'  n = %d;\n',n);
fprintf(fid,'  m = %d;\n',m);
fprintf(fid,'  nnz_jac_g = %d;\n',nnz_jac_g);
fprintf(fid,'  nnz_h_lag = %d;\n',nnz_h_lag);
fprintf(fid,'  index_style = TNLP::C_STYLE;\n  return true;\n}\n\n');

% bounds info method
fprintf(fid,'bool %s::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u)\n{\n',nlp_class_name);

for ii=1:n,
    fprintf(fid,'  x_l[%d] = %f;\n',ii-1,x_L(ii));
    fprintf(fid,'  x_u[%d] = %f;\n',ii-1,x_H(ii));
end
for ii=1:m,
    fprintf(fid,'  g_l[%d] = %f;\n',ii-1,g_L(ii));
    fprintf(fid,'  g_u[%d] = %f;\n',ii-1,g_H(ii));
end
fprintf(fid,'  return true;\n}\n\n');

% starting point
fprintf(fid,'bool %s::get_starting_point(Index n, bool init_x, Number* x, bool init_z, Number* z_L, Number* z_U, Index m, bool init_lambda, Number* lambda)\n{\n',nlp_class_name);
fprintf(fid,'  assert(init_x == true);\n  assert(init_z == false);\n  assert(init_lambda == false);\n\n');
for ii=1:n,
    fprintf(fid,'  x[%d] = %f;\n',ii-1,x0(ii));
end
fprintf(fid,'  return true;\n}\n\n');

% objective
fprintf(fid,'bool %s::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)\n{\n',nlp_class_name);
c = ccode(f);
% replace the default return value
c = regexprep(c,'t0','obj_value');
c = fix_params_c(c,params);
c = fix_dec_var_c(c,n);
fprintf(fid,'%s\n  return true;\n}\n\n',c);

% gradient of objective
fprintf(fid,'bool %s::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)\n{\n',nlp_class_name);
c=ccode(grad_f);
c = fix_params_c(c,params);
c = fix_dec_var_c(c,n);
% replace the extra indices
c = regexprep(c,']\[0\] =','] =');
fprintf(fid,'%s\n  return true;\n}\n\n',c);

% constraints
fprintf(fid,'bool %s::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)\n{\n',nlp_class_name);
c=ccode(g);
c = fix_params_c(c,params);
c = fix_dec_var_c(c,n);
% replace the extra indices
c = regexprep(c,']\[0\] =','] =');
fprintf(fid,'%s\n  return true;\n}\n\n',c);

% Jacobian of constraints
fprintf(fid,'bool %s::eval_jac_g(Index n, const Number* x, bool new_x, Index m, Index nele_jac, Index* iRow, Index *jCol,  Number* values)\n{\n  if (values == NULL) {\n',nlp_class_name);
for ii=1:nnz_jac_g,
    fprintf(fid,'    iRow[%d] = %d; jCol[%d] = %d;\n',ii-1,iRow(ii)-1,ii-1,jCol(ii)-1);
end
c = ccode(jac_g_values);
c = fix_params_c(c,params);
c = fix_dec_var_c(c,n);
c = regexprep(c,'jac_g_values\[0\]','  values');
fprintf(fid,'  }\n  else {\n%s\n  }\n  return true;\n}\n',c);

% Hessian of Lagrangian
fprintf(fid,'bool %s::eval_h(Index n, const Number* x, bool new_x, Number obj_factor, Index m, const Number* lambda, bool new_lambda, Index nele_hess, Index* iRow, Index* jCol, Number* values)\n{\n  if (values == NULL) {\n',nlp_class_name);
for ii=1:nnz_h_lag,
    fprintf(fid,'    iRow[%d] = %d; jCol[%d] = %d;\n',ii-1,iRow_h(ii)-1,ii-1,jCol_h(ii)-1);
end
c = ccode(h_lag_values);
c = fix_params_c(c,params);
c = fix_dec_var_c(c,n);
c = fix_lambda_c(c,m);
c = regexprep(c,'h_lag_values\[0\]','  values');
fprintf(fid,'  }\n  else {\n%s\n  }\n  return true;\n}\n',c);

fclose(fid);