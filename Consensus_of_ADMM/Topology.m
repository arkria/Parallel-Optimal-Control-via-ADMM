clear
clc
n = 100;
Np = 3;
rho = 10;
max_step = 3000;
epsi_rel = 10^-3;
epsi_abs = 10^-3;
rel_mat = eye(n,n-1)+rot90(eye(n,n-1),2);
[f, g] = GenerateTopology(rel_mat,Np);
[accuracy, empty_tag] = Centralized(f, g, rel_mat);
[z_out,tag,test_save, rho_mat, time_mat, A_set_save] = ADMM_DR_sy( f, g, rel_mat, rho, max_step, epsi_rel, epsi_abs, accuracy, empty_tag );
disp(['============= node number:', num2str(n),' ============='])
close all
figure(1)
plot(log10(test_save(:,1)))
hold on
plot(log10(test_save(:,2)))
plot(log10(test_save(:,3)))
plot(log10(test_save(:,4)))
legend('primal radius','dual radius','primal criterion','dual criterion')
figure(2)
accuracy = accuracy';z_out = z_out';
plot(abs(accuracy(:)), '-og');
hold on
plot(abs(z_out(:)),'*b');
ERR = abs(accuracy(:)-z_out(:));
plot(ERR,'r');
legend('real value','ADMM','error')