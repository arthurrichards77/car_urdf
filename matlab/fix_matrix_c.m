function s2 = fix_matrix_c(s1,mname,m,n)

s2 = s1;

% do it backwards so x11 doesn't become x[0] by matching x1
for ii=m:-1:1,
    for jj=n:-1:1,
        pat = sprintf('%s%d_%d',mname,ii,jj);
        rep = sprintf('%s[%d][%d]',mname,ii-1,jj-1);
        s2 = regexprep(s2,pat,rep);
    end
end