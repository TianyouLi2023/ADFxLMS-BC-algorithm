%-------------------%计算用户参数-------------------%
function [w_0, Phi_0, Sigma_w_k]=Calculate_user_coef(Num_WFS_source,w_L,A_narrow,t_Sequence,hh,PATH_P,J_G_length,N_VSR,Num_error,G_L_M_J,G_VSR,b_l_k)
VSR_Fx=zeros(Num_WFS_source*w_L,Num_WFS_source);
VSR_I=0.0000000001*eye(Num_WFS_source*w_L);
%VSR_I=zeros(Num_WFS_source*w_L);
sim_VSR=0;R_xx_VSR=zeros(Num_WFS_source*w_L,Num_WFS_source*w_L);
r_xd_VSR=zeros(Num_WFS_source*w_L,1);
xF_n_VSR = zeros(J_G_length,1);

for VSR_signal_kk=1:N_VSR
    
CH_VSR=randn(2*length(t_Sequence),1);CH_VSR=filter(hh,1,CH_VSR);
exp_VSR=CH_VSR(length(t_Sequence)+1:end,1);%忽略前端滤波卷积零值
exp_SIGNAL_VSR=A_narrow*exp_VSR;

    h_t_vis_error=PATH_P;
for ii=1:Num_error                    %循环后得到虚拟源到第ii个误差传声器之间的单位冲激响应,并计算期望信号
     p_error_Primary_VSR(:,ii)=filter((h_t_vis_error(ii,:))',1,exp_SIGNAL_VSR);   %计算各误差传声器上的期望信号
end 
 

for sim_VSR=1:1.1*w_L%计算滤波参考矩阵
    %VSR_tt
    %sim_VSR=sim_VSR+1;
    xF_n_VSR(2:end,1)=xF_n_VSR(1:end-1,1);xF_n_VSR(1,1)=exp_SIGNAL_VSR(sim_VSR,1);%更新滤波-x信号序列,用于计算VSR参数
    for secondy_ii=1:Num_WFS_source
        VSR_Fx( (secondy_ii-1)*w_L+2: secondy_ii*w_L    ,secondy_ii)=VSR_Fx( (secondy_ii-1)*w_L+1: secondy_ii*w_L-1    ,secondy_ii);
        VSR_Fx((secondy_ii-1)*w_L+1,secondy_ii)=(G_L_M_J(secondy_ii,:,secondy_ii))*xF_n_VSR;%自身的路径卷积
    end
end
d_M_n_VSR = p_error_Primary_VSR; %24个误差传声器上的期望信号
D_n_VSR = (d_M_n_VSR(sim_VSR,:)).'; 
R_xx_VSR=R_xx_VSR+(VSR_Fx*VSR_Fx.');
r_xd_VSR=r_xd_VSR+(VSR_Fx*D_n_VSR);  

end
R_xx_VSR=R_xx_VSR/(N_VSR);
r_xd_VSR=r_xd_VSR/(N_VSR);
w_0=-1*(R_xx_VSR+VSR_I)\r_xd_VSR;%plot(w_0);            
Phi_0=(G_VSR+0.000001*eye(w_L*Num_WFS_source))\w_0;%计算得到Wiener-Diffusion解
Sigma_w_k=zeros(Num_WFS_source,1);%计算用户参数-南大学报中文（22）式；
for Sigma_ii=1:Num_WFS_source
    for Sigma_jj=1:Num_WFS_source
        Sigma_w_k(Sigma_ii,1)=Sigma_w_k(Sigma_ii,1)+b_l_k(Sigma_ii,Sigma_jj)*(norm((Phi_0(   ((Sigma_ii-1)*w_L+1):Sigma_ii*w_L       ,1))-Phi_0(   ((Sigma_jj-1)*w_L+1):Sigma_jj*w_L  ,1),2))^2;
    end
end

%---------------------------------------------------%
