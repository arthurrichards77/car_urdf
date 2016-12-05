function s2 = fix_lambda_c(s1,m)

s2 = s1;

% do it backwards so x11 doesn't become x[0] by matching x1
for ii=m:-1:1,
    pat = sprintf('lambda%d',ii);
    rep = sprintf('lambda[%d]',ii-1);
    s2 = regexprep(s2,pat,rep);
end