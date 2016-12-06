close all
clear all

for kk=1:20,
  tic
  [status(kk),result] = system(sprintf('solveit %d',kk));
  ts(kk)=toc
end
assert(all(status==0))
M = [0*ts+1;1:numel(ts)]'
v = M\ts'
plot(1:numel(ts),ts,'o',1:numel(ts),M*v,'-')
title(sprintf('Offset = %f increment = %f',v(1),v(2)))