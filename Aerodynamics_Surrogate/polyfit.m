format long

fid = fopen('inputs.txt','rt');
C = textscan(fid,'%f');

AoA=C{1,1}(1:20);
mach=C{1,1}(21:40);
rpm_forward=C{1,1}(41:60);
rpm_lift=C{1,1}(61:80);
Cp_lift=C{1,1}(81:100);
Cp_forward=C{1,1}(101:120);
Ct_lift=C{1,1}(121:140);
Ct_forward=C{1,1}(141:160);

Xmatrix=[AoA mach rpm_forward rpm_lift Cp_lift Cp_forward Ct_lift Ct_forward];

x11=AoA.*AoA;
x22=mach.*mach;
x33=rpm_forward.*rpm_forward;
x44=rpm_lift.*rpm_lift;
x55=Cp_lift.*Cp_lift;
x66=Cp_forward.*Cp_forward;
x77=Ct_lift.*Ct_lift;
x88=Ct_forward.*Ct_forward;

x12=AoA.*mach;
x13=AoA.*rpm_forward;
x14=AoA.*rpm_lift;
x15=AoA.*Cp_lift;
x16=AoA.*Cp_forward;
x17=AoA.*Ct_lift;
x18=AoA.*Ct_forward;

x23=mach.*rpm_forward;
x24=mach.*rpm_lift;
x25=mach.*Cp_lift;
x26=mach.*Cp_forward;
x27=mach.*Ct_lift;
x28=mach.*Ct_forward;

x34=rpm_forward.*rpm_lift;
x35=rpm_forward.*Cp_lift;
x36=rpm_forward.*Cp_forward;
x37=rpm_forward.*Ct_lift;
x38=rpm_forward.*Ct_forward;

x45=rpm_lift.*Cp_lift;
x46=rpm_lift.*Cp_forward;
x47=rpm_lift.*Ct_lift;
x48=rpm_lift.*Ct_forward;

x56=Cp_lift.*Cp_forward;
x57=Cp_lift.*Ct_lift;
x58=Cp_lift.*Ct_forward;

x67=Cp_forward.*Ct_lift;
x68=Cp_forward.*Ct_forward;

x78=Ct_lift.*Ct_forward;

xdata1=[Xmatrix x11 x22 x33 x44 x55 x66 x77 x88];

xdata2=[xdata1 x12 x13 x14 x15 x16 x17 x18];

xdata3=[xdata2 x23 x24 x25 x26 x27 x28];

xdata4=[xdata3 x34 x35 x36 x37 x38];

xdata5=[xdata4 x45 x46 x47 x48];

xdata6=[xdata5 x56 x57 x58];

xdata7=[xdata6 x67 x68];

xdata=[xdata7 x78];

fun = @(x,xdata)x(1)+x(2)*xdata(:,1)+x(3)*xdata(:,2)+x(4)*xdata(:,3)+x(5)*xdata(:,4)+x(6)*xdata(:,5)+x(7)*xdata(:,6)+x(8)*xdata(:,7)+x(9)*xdata(:,8)+x(10)*xdata(:,9) ...
    +x(11)*xdata(:,10)+x(12)*xdata(:,11)+x(13)*xdata(:,12)+x(14)*xdata(:,13)+x(15)*xdata(:,14)+x(16)*xdata(:,15)+x(17)*xdata(:,16)+x(18)*xdata(:,17)+x(19)*xdata(:,18)+x(20)*xdata(:,19) ...
    +x(21)*xdata(:,20)+x(22)*xdata(:,21)+x(23)*xdata(:,22)+x(24)*xdata(:,23)+x(25)*xdata(:,24)+x(26)*xdata(:,25)+x(27)*xdata(:,26)+x(28)*xdata(:,27)+x(29)*xdata(:,28)+x(30)*xdata(:,29)... 
    +x(31)*xdata(:,30)+x(32)*xdata(:,31)+x(33)*xdata(:,32)+x(34)*xdata(:,33)+x(35)*xdata(:,34)+x(36)*xdata(:,35)+x(37)*xdata(:,36)+x(38)*xdata(:,37)+x(39)*xdata(:,38)+x(40)*xdata(:,39) ...
    +x(41)*xdata(:,40)+x(42)*xdata(:,41)+x(43)*xdata(:,42)+x(44)*xdata(:,43)+x(45)*xdata(:,44);

x0 = [-10.0,-1.0,20.0,-30.0,534.0,-2.0,300.0,-5.0,500.0,90.0 ...
    ,1.0,11.0,600.0,900.5,70.0,9.0,111.0,8.0,145.0,35.0 ...
    ,0.26,-3.0,0.6,-2.8,4.7,10.4,5.9,9.88,-4.8,7.9,9.3...
    ,8.4,8.0,-100.23,-40.56,2.8,6.9,7.0,10.75,11.9,0.88...
    ,-0.9,-23.0,44.0,505.8,4.8];

x0 = x0';

fid = fopen('CL_results.txt','rt');
CL = textscan(fid,'%f');

Y=CL{1,1};

options = optimoptions('lsqcurvefit','Algorithm','trust-region-reflective')%,'TolFun',1e-30,'FinDiffRelStep',1.038805897316893e-11);

lb = [];
ub = [];

[x,resnorm,residual,exitflag,output] = lsqcurvefit(fun,x0,xdata,Y,lb,ub,options)

times = linspace(0,5);

i=0;

times=zeros(length(Y));

while i<length(Y)
    times(i+1)=i+1;
    i=i+1;
end

figure(1)
hold on
plot(times(:,1),Y,'ko')
plot(fun(x,xdata),'b-')
legend('CL Data','Fitted exponential')
title('Fitted Curve')
hold off
