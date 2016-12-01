param Ncars>0;
param Narcs>0;
param Ntau = 7;
param Vinit{1..Ncars}>=0;
param Vmax{1..Ncars,1..Narcs}>=0;
param Larcs{1..Ncars,1..Narcs}>0;

param Ncross>0;
param Dcross{1..Ncross,1..4} integer; # each row is [car 1, arc 1, car 2, arc 2]

var Twait{1..Ncars,1..Narcs}>=0;
var Tdrive{i in 1..Ncars,j in 1..Narcs}>=Larcs[i,j]/Vmax[i,j];
var Vnode{i in 1..Ncars,j in 1..(Narcs+1)}>=0;

param Wdrive = 0.3;

minimize cost: sum{i in 1..Ncars, j in 1..Narcs} (Twait[i,j]+(1+Wdrive)*Tdrive[i,j]);

subject to fixVinit{i in 1..Ncars}: Vnode[i,1] = Vinit[i];
subject to fixVwait{i in 1..Ncars, j in 1..Narcs}: Vnode[i,j]*Twait[i,j] = 0;

subject to fixVterm{i in 1..Ncars}: Vnode[i,Narcs+1] = 0;

subject to minV{i in 1..Ncars, j in 1..Narcs, k  in 1..Ntau}: Vnode[i,j]*(1-k/Ntau)^2 + Vnode[i,j+1]*(k/Ntau)^2 + (6*(Larcs[i,j]/Tdrive[i,j]) - 2*Vnode[i,j] - 2*Vnode[i,j+1])*(k/Ntau)*(1-k/Ntau) >= 0;
subject to maxV{i in 1..Ncars, j in 1..Narcs, k  in 1..Ntau}: Vnode[i,j]*(1-k/Ntau)^2 + Vnode[i,j+1]*(k/Ntau)^2 + (6*(Larcs[i,j]/Tdrive[i,j]) - 2*Vnode[i,j] - 2*Vnode[i,j+1])*(k/Ntau)*(1-k/Ntau) <= Vmax[i,j];

# car 1 can't enter arc 2 until car 1 has left it
#subject to avoid: Twait[1,1]+Tdrive[1,1]+Twait[1,2] >= Twait[2,1]+Tdrive[2,1]+Twait[2,2]+Tdrive[2,2];
#subject to avoid: Twait[2,1]+Tdrive[2,1]+Twait[2,2] >= Twait[1,1]+Tdrive[1,1]+Twait[1,2]+Tdrive[1,2];

var lam{1..Ncross} >=0, <=1;
subject to avoid{a in 1..Ncross: Dcross[a,1]<>0}: lam[a]*(sum{j in 1..Dcross[a,2]} Twait[Dcross[a,1],j] + sum{j in 1..Dcross[a,2]-1} Tdrive[Dcross[a,1],j] - sum{j in 1..Dcross[a,4]} Twait[Dcross[a,3],j] - sum{j in 1..Dcross[a,4]} Tdrive[Dcross[a,3],j]) + (1-lam[a])*(sum{j in 1..Dcross[a,4]} Twait[Dcross[a,3],j] + sum{j in 1..Dcross[a,4]-1} Tdrive[Dcross[a,3],j] - sum{j in 1..Dcross[a,2]} Twait[Dcross[a,1],j] - sum{j in 1..Dcross[a,2]} Tdrive[Dcross[a,1],j])>=0;