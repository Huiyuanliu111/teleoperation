close all;
run('DATA.m')
index = DATAm(:,1);
pandatime = DATAm(:,2);
remotetime = DATAm(:,3);
qmsr_delta = DATAm(:,4:10);
q_des_delta = DATAm(:,11:17);
dq_msr = DATAm(:,18:24);
dq_des = DATAm(:,25:31);
tau_ext_msr = DATAm(:,32:38);
tau_ext_des = DATAm(:,39:45);
q_des_deltaF = DATAm(:,46:52);
dq_desF = DATAm(:,53:59);
f_ext_msr = DATAm(:,60:65);
f_ext_des = DATAm(:,66:71);
power_des = DATAm(:,72:74);
power_msr = DATAm(:,75:77);
energy_des = DATAm(:,78:80);
energy_msr = DATAm(:,81:83);

figure();
plot(index(1:59000), power_des(1:59000,:));
title('Leader Power')

figure();
plot(index(1:59000), energy_des(1:59000,:));
title('Leader Energy')

figure();
plot(index(1:59000), power_msr(1:59000,:));

figure();
plot(index(1:59000), energy_msr(1:59000,:));