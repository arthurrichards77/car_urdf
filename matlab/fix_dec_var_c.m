function s2 = fix_dec_var_c(s1,n)

s2 = s1;

% do it backwards so x11 doesn't become x[0] by matching x1
for ii=n:-1:1,
    pat = sprintf('x%d',ii);
    rep = sprintf('x[%d]',ii-1);
    s2 = regexprep(s2,pat,rep);
end