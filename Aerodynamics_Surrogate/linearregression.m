format short g

%% Obtain input variables

fid = fopen('inputs.txt','rt');
C = textscan(fid,'%f');

%% Create X matrix from inputs

AoA=C{1,1}(1:10);
mach=C{1,1}(11:20);
rpm_forward=C{1,1}(21:30);
rpm_lift=C{1,1}(31:40);
Cp_lift=C{1,1}(41:50);
Cp_forward=C{1,1}(51:60);
Ct_lift=C{1,1}(61:70);
Ct_forward=C{1,1}(71:80);

Xmatrix=[AoA mach rpm_forward rpm_lift Cp_lift Cp_forward Ct_lift Ct_forward];

%% Obtain CL values calculated in VSPAERO and create array

fid = fopen('CL_results.txt','rt');
CL = textscan(fid,'%f');

Y=CL{1,1};

%% Create table for fitlm
tbl = table(Xmatrix(:,1),Xmatrix(:,2),Xmatrix(:,3),Xmatrix(:,4),Xmatrix(:,5),Xmatrix(:,6),Xmatrix(:,7),Xmatrix(:,8),Y, ...
    'VariableNames', ...
    {'AoA','mach','rpm_forward','rpm_lift','Cp_lift','Cp_forward','Ct_lift','Ct_forward','CL'});

%% Execute fitlm and create plot

%mdl=fitlm(tbl,'CL~mach+AoA^2+AoA*mach+rpm_forward*Cp_forward+rpm_forward*Ct_forward+rpm_lift*Cp_lift+rpm_lift*Ct_lift+rpm_forward*mach+rpm_lift*mach+rpm_forward^2+rpm_lift^2+AoA*rpm_forward+AoA*rpm_lift')

mdl=fitlm(tbl,'CL~mach+AoA^2+AoA*Ct_forward+AoA*rpm_lift+rpm_forward*Ct_forward')

%mdl=fitlm(Xmatrix,Y,'quadratic')


coeff=mdl.Coefficients.Estimate;

plot(mdl)


