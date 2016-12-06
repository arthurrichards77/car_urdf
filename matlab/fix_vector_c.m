function s2 = fix_vector_c(s1,vname,m)

s2 = s1;

% do it backwards so x11 doesn't become x[0] by matching x1
for ii=m:-1:1,
    pat = sprintf('%s%d',vname,ii);
    rep = sprintf('%s[%d]',vname,ii-1);
    s2 = regexprep(s2,pat,rep);
end