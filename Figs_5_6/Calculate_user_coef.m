function [w_0, Phi_0, Sigma_w_k]=Calculate_user_coef(Num_WFS_source,w_L,A_narrow,t_Sequence,hh,x_0,x_array_error,Fs,c,J_G_length,N_VSR,Num_error,G_L_M_J,G_VSR,b_l_k)

    % "Calculate_user_coef.m" is used to calculate the user coefficients of the MDFxLMS-VSR algorithm. 
    % The algorithm process of the MDFxLMS-VSR algorithm can be found in reference "Chu, Y., Mak, C., Cai, C., and Wu, M. (2021).
    %  A new variable spatial regularized FxLMS algorithm for diffusion active noise control,  Journal of Nanjing University (Natural Science). 57, 683-689."

VSR_Fx=zeros(Num_WFS_source*w_L,Num_WFS_source);
VSR_I=0.0000001*eye(Num_WFS_source*w_L);
R_xx_VSR=zeros(Num_WFS_source*w_L,Num_WFS_source*w_L);
r_xd_VSR=zeros(Num_WFS_source*w_L,1);
xF_n_VSR = zeros(J_G_length,1);

for VSR_signal_kk=1:N_VSR 

CH_VSR=randn(2*length(t_Sequence),1);CH_VSR=filter(hh,1,CH_VSR); 
exp_VSR=CH_VSR(length(t_Sequence)+1:end,1);
exp_SIGNAL_VSR=A_narrow*exp_VSR; % Noise source of offline experiments for Wiener solutions calculation

for ii=1:Num_error                    
    range_error = x_array_error(:,ii);
    range_dr(ii,1) = sqrt( (x_0(1,1)-range_error(1,1))^2 + (x_0(2,1)-range_error(2,1))^2 + (x_0(3,1)-range_error(3,1))^2);
    h_t_vis_error(ceil(range_dr(ii,1)*Fs/c),ii) = 1/range_dr(ii,1);       
    p_error_Primary_VSR(:,ii)=filter(h_t_vis_error(:,ii),1,exp_SIGNAL_VSR); % The primary noise at the error microphones
end 

for sim_VSR=1:1.1*w_L % Update the filtered-reference signal matrix
    xF_n_VSR(2:end,1)=xF_n_VSR(1:end-1,1);xF_n_VSR(1,1)=exp_SIGNAL_VSR(sim_VSR,1);
    for secondy_ii=1:Num_WFS_source
        VSR_Fx( (secondy_ii-1)*w_L+2: secondy_ii*w_L    ,secondy_ii)=VSR_Fx( (secondy_ii-1)*w_L+1: secondy_ii*w_L-1    ,secondy_ii);
        VSR_Fx((secondy_ii-1)*w_L+1,secondy_ii)=(G_L_M_J(secondy_ii,:,secondy_ii))*xF_n_VSR;
    end
end
d_M_n_VSR = p_error_Primary_VSR;     
D_n_VSR = (d_M_n_VSR(sim_VSR,:)).';  
R_xx_VSR=R_xx_VSR+(VSR_Fx*VSR_Fx.'); 
r_xd_VSR=r_xd_VSR+(VSR_Fx*D_n_VSR);    
end

R_xx_VSR=R_xx_VSR/(N_VSR);
r_xd_VSR=r_xd_VSR/(N_VSR);
w_0=-1*(R_xx_VSR+VSR_I)\r_xd_VSR;          
Phi_0=(G_VSR+0.000001*eye(w_L*Num_WFS_source))\w_0; %  Calculate the Wiener-Diffusion solutions, Refer to Equation (21) in (Chu et al., 2021) 
Sigma_w_k=zeros(Num_WFS_source,1); % The user coefficients of the MDFxLMS-VSR Algorithm, Refer to Equation (22) in (Chu et al., 2021)
for Sigma_ii=1:Num_WFS_source
    for Sigma_jj=1:Num_WFS_source
        Sigma_w_k(Sigma_ii,1)=Sigma_w_k(Sigma_ii,1)+b_l_k(Sigma_ii,Sigma_jj)*(norm((Phi_0(((Sigma_ii-1)*w_L+1):Sigma_ii*w_L,1))-Phi_0(((Sigma_jj-1)*w_L+1):Sigma_jj*w_L,1),2))^2;
    end
end
