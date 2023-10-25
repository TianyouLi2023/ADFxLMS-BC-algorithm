function [w_L,Num_WFS_source,Wn_Global_GD_ATCC,Wn_Global_GD,Wn_Global_C,Wn_Global_DC,Wn_Global_MD,Wn_Global_MD_VSR,CDLMS_E_n,DCLMS_E_n,MDLMS_E_n,GDLMS_E_n,MDLMS_E_n_VSR,GDLMS_E_n_ATCC,SYSTEM_d_n]=...
         Main2_JASA_Single_Simulation(signal_select)
    % The sub-function "Main2_JASA_Single_Simulation" can execute a noise reduction simulation experiment.
    % Input parameter: signal_select=1, which denotes broadband noise reduction simulation experiment. Users can set multiple primary noise signals as needed. 
    % Refer to our previous work "Distributed Active Noise Control Based on an Augmented Diffusion FxLMS Algorithm", which presents different primary noise configuration scenarios (Section V. Simulations). 
    
    % Output parameter: 
    % (1) w_L denotes the length of the control filters;
    % (2) Num_WFS_source denotes the number of noise control channel;
    % (3) Wn_Global_GD_ATCC, Wn_Global_GD, Wn_Global_C,Wn_Global_DC,Wn_Global_MD,Wn_Global_MD_VSR denote the global control filter of 
    % the ADFxLMS-BC, ADFxLMS, CFxLMS, DCFxLMS, MDFxLMS, MDFxLMS-VSR algorithms, respectively. 
    % (4) CDLMS_E_n,DCLMS_E_n,MDLMS_E_n,GDLMS_E_n,MDLMS_E_n_VSR,GDLMS_E_n_ATCC denote the global error signals of 
    % the CFxLMS, DCFxLMS, MDFxLMS, ADFxLMS, MDFxLMS-VSR, ADFxLMS-BC algorithms, respectively. 
    % (5) SYSTEM_d_n denote the global primary noise. 
%% Basic parameters
c = 340;   % sound speed
w_L = 120; % The length of the control filters
s_L = 50;  % The length of the secondary paths
f_L = 50;  % The length of the primary paths
SNR_en=30; % The signal-to-noise ratio (SNR)

if signal_select==1 % Broadband noise reduction simulation
mu_CLMS = 1.3;      % The step size of the CFxLMS algorithm
mu_MDLMS = 0.5;     % The step size of the MDFxLMS algorithm
mu_MDLMS_VSR = 0.7; % The step size of the MDFxLMS-VSR algorithm
mu_BDLMS_NBC = 1.2; % The step size of the ADFxLMS algorithm
mu_BDLMS = 2.4;     % The step size of the DCFxLMS and ADFxLMS-BC algorithms

Ts = 3;    % The time length of Free-field simulations (second)
Fs = 4000; % Sampling Frequency (Hz)

f_left = 100;f_right = 1500; % 100-order FIR bandpass filter
Narrow_order=100;
hh=fir1(Narrow_order,[f_left/(Fs/2) f_right/(Fs/2)]);

A_narrow = 2;t_Sequence = (0:1/Fs:Ts-1/Fs).'; % The bandpass-filtered (100 Hz – 1500 Hz) white Gaussian noise
CH_Source=randn(2*length(t_Sequence),1);CH_Source=filter(hh,1,CH_Source);
exp_narrow=CH_Source(length(t_Sequence)+1:end,1);
exp_Sequence=A_narrow*exp_narrow;
end
      
%% Environmental parameters
R_array = 1.2;       % The radii of the circular secondary loudspeaker arrays
R_array_error = 1.0; % The radii of the circular error microphone arrays
Num_WFS_source = 10; % The number of secondary loudspeakers
Num_error = 10;      % The number of error microphones

x_array_mid = [0;0;0];   % The position of the secondary loudspeakers and error microphones
x_0 = [0;2.5;0];         
ANGle_WFS=(360/180)*pi;  
ANGle_WFS0=(0/180)*pi;   
ANGle_WFS1=(360/180)*pi; 
Angle_WFS_source = (ANGle_WFS0:ANGle_WFS/Num_WFS_source:ANGle_WFS1-ANGle_WFS/Num_WFS_source)';

x1_array_error = x_array_mid(1,1)*ones(Num_error,1)+R_array_error*cos(Angle_WFS_source);
x2_array_error = x_array_mid(2,1)*ones(Num_error,1)-R_array_error*sin(Angle_WFS_source);
x3_array_error = x_array_mid(3,1)*zeros(Num_error,1);
x_array_error = [x1_array_error';x2_array_error';x3_array_error'];

x1_array_WFS_source = x_array_mid(1,1)*ones(Num_WFS_source,1)+R_array*cos(Angle_WFS_source);
x2_array_WFS_source = x_array_mid(2,1)*ones(Num_WFS_source,1)-R_array*sin(Angle_WFS_source);
x3_array_WFS_source = x_array_mid(2,1)*zeros(Num_WFS_source,1);
x_array_source = [x1_array_WFS_source';x2_array_WFS_source';x3_array_WFS_source'];

Link_Matrix=zeros(Num_WFS_source,Num_WFS_source);%The communication links
Link_Matrix(1,1)=1;  Link_Matrix(1,2)=1;  Link_Matrix(1,10)=1;
Link_Matrix(2,1)=1;  Link_Matrix(2,2)=1;  Link_Matrix(2,3)=1; 
Link_Matrix(3,2)=1;  Link_Matrix(3,3)=1;  Link_Matrix(3,4)=1;Link_Matrix(6,10)=1;Link_Matrix(10,6)=1;
Link_Matrix(4,3)=1;  Link_Matrix(4,4)=1;  Link_Matrix(4,5)=1;Link_Matrix(2,7)=1;Link_Matrix(7,2)=1;
Link_Matrix(5,4)=1;  Link_Matrix(5,5)=1;  Link_Matrix(5,6)=1; 
Link_Matrix(6,5)=1;  Link_Matrix(6,6)=1;  Link_Matrix(6,7)=1;
Link_Matrix(7,6)=1;  Link_Matrix(7,7)=1;  Link_Matrix(7,8)=1;
Link_Matrix(8,7)=1;  Link_Matrix(8,8)=1;  Link_Matrix(8,9)=1;
Link_Matrix(9,8)=1;  Link_Matrix(9,9)=1;  Link_Matrix(9,10)=1;
Link_Matrix(10,9)=1; Link_Matrix(10,10)=1;Link_Matrix(10,1)=1;

Link_VSR=zeros(Num_WFS_source,Num_WFS_source);%The fixed combination parameters based on the Metropolis rule
a_l_k=zeros(Num_WFS_source,Num_WFS_source);
for Link_ii=1:Num_WFS_source
    for Link_jj=1:Num_WFS_source
        if (Link_ii~=Link_jj)&&(Link_Matrix(Link_ii,Link_jj)~=0)
           Link_VSR(Link_ii,Link_jj)=1/max((sum(Link_Matrix(Link_ii,:))),(sum(Link_Matrix(Link_jj,:))));
        end
    end
end
for Link_ii=1:Num_WFS_source
    for Link_jj=1:Num_WFS_source
        if Link_ii==Link_jj
           Link_VSR(Link_ii,Link_jj)=1-sum(Link_VSR(Link_ii,:));
        end
    end
end

range_dr = zeros(Num_error,1);                             
h_t_vis_error = zeros(f_L,Num_error);                      
p_error_Primary_t = (zeros(Num_error,Ts*Fs)).';            
for ii=1:Num_error                    
    range_error = x_array_error(:,ii);
    range_dr(ii,1) = sqrt( (x_0(1,1)-range_error(1,1))^2 + (x_0(2,1)-range_error(2,1))^2 + (x_0(3,1)-range_error(3,1))^2);
    h_t_vis_error(ceil(range_dr(ii,1)*Fs/c),ii) = 1/(4*pi*range_dr(ii,1)); % The primary paths in free-field simulations      
    p_error_Primary_t(:,ii)=filter(h_t_vis_error(:,ii),1,exp_Sequence);    % The primary noise at error microphones
end 

jvli_second_error=zeros(Num_WFS_source,Num_error);
for ii=1:Num_error                         
    location_error = x_array_error(:,ii);  
    for jj=1:Num_WFS_source                
        location_secondy = x_array_source(:,jj);
        jvli_second_error(jj,ii) = sqrt( ( location_secondy(1,1)-location_error(1,1) )^2 + (location_secondy(2,1)-location_error(2,1))^2 + (location_secondy(3,1)-location_error(3,1))^2 );    
    end 
end
G_L_M_J = zeros(Num_error,s_L,Num_WFS_source);
for LL=1:Num_WFS_source    
    for MM=1:Num_error     
    G_L_M_J(MM,ceil(jvli_second_error(LL,MM)*Fs/c),LL) = 1/(4*pi*jvli_second_error(LL,MM)); % The secondary paths in free-field simulations 
    end   
end

%% Algorithm parameters
d_M_n = p_error_Primary_t;                        % Primary Noise
xF_n = zeros(s_L,1);                              % Reference signal which is used to calculate the filtered-reference signal 
R_n = zeros(Num_WFS_source,Num_error*w_L);        % Global filtered-reference signal matrix
R_n_nomal  = zeros(Num_error,Num_WFS_source*w_L); % Global normalized filtered-reference signal matrix
d_n = zeros(Num_WFS_source,Ts*Fs);                % Primary Noise participating in iteration

%-------------------------------Variable Definition of the ADFxLMS-BC Algorithm---------------------------------%
En_G_ATCC = zeros(Num_WFS_source,Ts*Fs);          % The error signal
Wn_Global_GD_ATCC = zeros(Num_WFS_source*w_L,1);  % The global control filter
An_G_ATCC = cell(Num_WFS_source,1);               % The adaptation result
for G_ii_ATCC = 1:Num_WFS_source
    neigh_num = sum(Link_Matrix(:,G_ii_ATCC));
    An_G_ATCC{G_ii_ATCC,1}=cell(2,neigh_num);
    for Nonzero_index=1:neigh_num
        An_G_ATCC{G_ii_ATCC,1}{1,Nonzero_index}=zeros(w_L,1);
    end
    neigh_index=0;
    for Nonzero_index=1:Num_WFS_source
        if Link_Matrix(Nonzero_index,G_ii_ATCC)==1
           neigh_index=neigh_index+1;
           An_G_ATCC{G_ii_ATCC,1}{2,neigh_index}=Nonzero_index;
        end
    end
end

Wn_G_ATCC = cell(Num_WFS_source,1);               % The combination result
for G_ii=1:Num_WFS_source
    neigh_num=sum(Link_Matrix(:,G_ii));
    Wn_G_ATCC{G_ii,1}=cell(2,neigh_num);
    for Nonzero_index=1:neigh_num
        Wn_G_ATCC{G_ii,1}{1,Nonzero_index}=zeros(w_L,1);
    end
    neigh_index=0;
    for Nonzero_index=1:Num_WFS_source
        if Link_Matrix(Nonzero_index,G_ii)==1
           neigh_index=neigh_index+1;
           Wn_G_ATCC{G_ii,1}{2,neigh_index}=Nonzero_index;
        end
    end
end

%---------------------------------Variable Definition of the ADFxLMS Algorithm-----------------------------------%
En_G = zeros(Num_WFS_source,Ts*Fs);         % The error signal
Wn_Global_GD = zeros(Num_WFS_source*w_L,1); % The global control filter
An_G = cell(Num_WFS_source,1);              % The adaptation result
for G_ii=1:Num_WFS_source
    neigh_num=sum(Link_Matrix(:,G_ii));
    An_G{G_ii,1}=cell(2,neigh_num);
    for Nonzero_index=1:neigh_num
        An_G{G_ii,1}{1,Nonzero_index}=zeros(w_L,1);
    end
    neigh_index=0;
    for Nonzero_index=1:Num_WFS_source
        if Link_Matrix(Nonzero_index,G_ii)==1
           neigh_index=neigh_index+1;
           An_G{G_ii,1}{2,neigh_index}=Nonzero_index;
        end
    end
end

Wn_G = cell(Num_WFS_source,1);              % The combination result
for G_ii=1:Num_WFS_source
    neigh_num=sum(Link_Matrix(:,G_ii));
    Wn_G{G_ii,1}=cell(2,neigh_num);
    for Nonzero_index=1:neigh_num
        Wn_G{G_ii,1}{1,Nonzero_index}=zeros(w_L,1);
    end
    neigh_index=0;
    for Nonzero_index=1:Num_WFS_source
        if Link_Matrix(Nonzero_index,G_ii)==1
           neigh_index=neigh_index+1;
           Wn_G{G_ii,1}{2,neigh_index}=Nonzero_index;
        end
    end
end

%---------------------------------Variable Definition of the CFxLMS Algorithm-----------------------------------%
En_C = zeros(Num_WFS_source,Ts*Fs);         % The error signal
Wn_Global_C = zeros(Num_WFS_source*w_L,1);  % The global control filter

%--------------------------------Variable Definition of the DCFxLMS Algorithm-----------------------------------%
En_DC = zeros(Num_WFS_source,Ts*Fs);        % The error signal
Wn_DC = zeros(w_L,Num_WFS_source);          % The decentralized control filters
Wn_Global_DC = zeros(Num_WFS_source*w_L,1); % The global control filter

%--------------------------------Variable Definition of the MDFxLMS Algorithm-----------------------------------%
En_MD = zeros(Num_WFS_source,Ts*Fs);        % The error signal
An_MD = zeros(w_L,Num_WFS_source);          % The adaptation result
Wn_MD = zeros(w_L,Num_WFS_source);          % The combination result
Wn_Global_MD = zeros(Num_WFS_source*w_L,1); % The global control filter

%------------------------------Variable Definition of the MDFxLMS-vsr Algorithm---------------------------------%
% Refer to (Chu et al., 2021)-"Chu, Y., Mak, C., Cai, C., and Wu, M. (2021). "A new variable spatial regularized 
% FxLMS algorithm for diffusion active noise control,” Journal of Nanjing University (Natural Science). 57, 683-689."
En_MD_VSR = zeros(Num_WFS_source,Ts*Fs);        % The error signal
An_MD_VSR = zeros(w_L,Num_WFS_source);          % The adaptation result
Wn_MD_VSR = zeros(w_L,Num_WFS_source);          % The combination result
Wn_Global_MD_VSR = zeros(Num_WFS_source*w_L,1); % The global control filter

N_VSR = 120; % The number of offline experiments for Wiener solutions calculation
lamda_k_n=zeros(Num_WFS_source,1);      
alfa_k_n=zeros(Num_WFS_source,1);       
beta_k=0.01*ones(Num_WFS_source,1);     
kesi_vsr=0.001;                         
yiks_k=kesi_vsr*ones(Num_WFS_source,1); 
kapa_k=0.01*ones(Num_WFS_source,1);     
a_l_k_VSR=zeros(Num_WFS_source,Num_WFS_source);
b_l_k=Link_VSR;
for Link_ii=1:Num_WFS_source
    for Link_jj=1:Num_WFS_source
        if Link_ii==Link_jj
           a_l_k(Link_ii,Link_jj)=1-kesi_vsr+b_l_k(Link_ii,Link_jj)*kesi_vsr;
        else 
           a_l_k(Link_ii,Link_jj)=kesi_vsr*b_l_k(Link_ii,Link_jj);
        end
    end
end
G_VSR=kron(a_l_k,eye(w_L));
if signal_select==1 % The user coefficients of the MDFxLMS-VSR Algorithm, Refer to Equation (22) in (Chu et al., 2021)
[~, ~, Sigma_w_k]=Calculate_user_coef(Num_WFS_source,w_L,A_narrow,t_Sequence,hh,x_0,x_array_error,Fs,c,s_L,N_VSR,Num_error,G_L_M_J,G_VSR,b_l_k);
end

%% Algorithm iteration
sim_kk = 0; % The current number of the completed iterations
for sim_tt = 0:1/Fs:Ts-1/Fs % The current simulation time (second)
    sim_kk = sim_kk+1;
    xF_n(2:end,1) = xF_n(1:end-1,1);xF_n(1,1) = exp_Sequence(sim_kk,1); % Update the reference signal
    
for error_jj=1:Num_WFS_source % Update the global filtered-reference signal matrix
        for second_ii=1:Num_error
        R_n(error_jj,(second_ii-1)*w_L+2:(second_ii-1)*w_L+w_L)=...
        R_n(error_jj,(second_ii-1)*w_L+1:(second_ii-1)*w_L+w_L-1); 
        R_n(error_jj,(second_ii-1)*w_L+1)=(G_L_M_J(error_jj,:,second_ii))*xF_n;
        end
end

for error_jj=1:Num_error % Update the normalized global filtered-reference signal matrix
        for second_ii=1:Num_WFS_source
        deltaa=120;
        R_n_nomal(error_jj,(second_ii-1)*w_L+1:(second_ii-1)*w_L+w_L)=...
        R_n(error_jj,(second_ii-1)*w_L+1:(second_ii-1)*w_L+w_L)/...
        (deltaa+(norm(R_n(error_jj,(second_ii-1)*w_L+1:(second_ii-1)*w_L+w_L),2)^2));    
        end
end
D_n = (d_M_n(sim_kk,:)).'; % Update the primary noise
d_n(:,sim_kk) = D_n;

%-----------------------------------------Iteration of the ADFxLMS-BC Algorithm-------------------------------------------%
for ii=1:Num_WFS_source % Update the global control filter
    for jj=1:sum(Link_Matrix(:,ii))
    if Wn_G_ATCC{ii,1}{2,jj}==ii
    Wn_Global_GD_ATCC((ii-1)*w_L+1:ii*w_L,1)=Wn_G_ATCC{ii,1}{1,jj}; 
    end
    end
end

Yn_G_ATCC=R_n*Wn_Global_GD_ATCC; % Calculate the anti-noise signal at the error microphones
En_G_ATCC(:,sim_kk) = D_n+awgn(Yn_G_ATCC,SNR_en,'measured'); % Calculate the error signal

for adaptG_ii=1:Num_WFS_source % Step. 1 - Neighborhood-wide adaptation 
    Adapt_phase_ii=zeros(sum(Link_Matrix(:,adaptG_ii))*w_L,1); 
    R_Adapt_ii_AD=zeros(1,sum(Link_Matrix(:,adaptG_ii))*w_L);

        for G_jj=1:sum(Link_Matrix(:,adaptG_ii))
            ww=An_G_ATCC{adaptG_ii,1}{2,G_jj};
            Adapt_phase_ii((G_jj-1)*w_L+1:(G_jj)*w_L,1)=Wn_G_ATCC{adaptG_ii,1}{1,G_jj};
            R_Adapt_ii_AD(1,(G_jj-1)*w_L+1:(G_jj)*w_L)=R_n_nomal((adaptG_ii-1)+1,((ww-1)*w_L+1):(ww*w_L));
        end

        Adapt_phase_ii=Adapt_phase_ii-mu_BDLMS* R_Adapt_ii_AD' *En_G_ATCC(adaptG_ii,sim_kk);

        for G_jj=1:sum(Link_Matrix(:,adaptG_ii))
            An_G_ATCC{adaptG_ii,1}{1,G_jj}=Adapt_phase_ii((G_jj-1)*w_L+1:(G_jj)*w_L,1);
        end
end

    for comG_ii=1:Num_WFS_source % Step. 2 - Node-specific combination in the forward communication    
        for C_jj=1:sum(Link_Matrix(:,comG_ii)) 
            if Wn_G_ATCC{comG_ii,1}{2,C_jj}==comG_ii
            Pass_W_n_ATCC=zeros(w_L,1);
            nei_sum=0;
            for nei_kk=1:sum(Link_Matrix(:,comG_ii))
                for nei_kk_ll=1:sum(Link_Matrix(:,Wn_G_ATCC{comG_ii,1}{2,nei_kk}))
                    if Wn_G_ATCC{Wn_G_ATCC{comG_ii,1}{2,nei_kk},1}{2,nei_kk_ll}==Wn_G_ATCC{comG_ii,1}{2,C_jj}
                       nei_sum=nei_sum+1;
                        Pass_W_n_ATCC=Pass_W_n_ATCC+An_G_ATCC{Wn_G_ATCC{comG_ii,1}{2,nei_kk},1}{1,nei_kk_ll}*Link_VSR(comG_ii,Wn_G_ATCC{comG_ii,1}{2,nei_kk});
                    end
                end 
            end  

            Wn_G_ATCC{comG_ii,1}{1,C_jj}=Pass_W_n_ATCC;
            end
        end
    end 

for ii=1:Num_WFS_source% Step. 3 - Neighborhood-wide reconfiguration in the backward communication
    for jj=1:sum(Link_Matrix(:,ii))
    jJ=Wn_G_ATCC{ii,1}{2,jj};
    for kk=1:sum(Link_Matrix(:,jJ))
        if Wn_G_ATCC{jJ,1}{2,kk}==jJ
           Wn_G_ATCC{ii,1}{1,jj}=Wn_G_ATCC{jJ,1}{1,kk};
        end
    end
    end
end

%-------------------------------------------Iteration of the ADFxLMS Algorithm---------------------------------------------%
for ii=1:Num_WFS_source % Update the global control filter
    for jj=1:sum(Link_Matrix(:,ii))
    if Wn_G{ii,1}{2,jj}==ii
    Wn_Global_GD((ii-1)*w_L+1:ii*w_L,1)=Wn_G{ii,1}{1,jj}; 
    end
    end
end

Yn_G=R_n*Wn_Global_GD; % Calculate the anti-noise signal at the error microphones
En_G(:,sim_kk) = D_n+awgn(Yn_G,SNR_en,'measured'); % Calculate the error signal

    for adaptG_ii=1:Num_WFS_source % Step. 1 - Neighborhood-wide adaptation
        Adapt_phase_ii=zeros(sum(Link_Matrix(:,adaptG_ii))*w_L,1); 
        R_Adapt_ii_AD=zeros(1,sum(Link_Matrix(:,adaptG_ii))*w_L);

        for G_jj=1:sum(Link_Matrix(:,adaptG_ii))
            Adapt_phase_ii((G_jj-1)*w_L+1:(G_jj)*w_L,1)=Wn_G{adaptG_ii,1}{1,G_jj};
            ww=An_G{adaptG_ii,1}{2,G_jj};
            R_Adapt_ii_AD(1,(G_jj-1)*w_L+1:(G_jj)*w_L)=R_n_nomal((adaptG_ii-1)+1,((ww-1)*w_L+1):(ww*w_L));
        end
        Adapt_phase_ii=Adapt_phase_ii-mu_BDLMS_NBC* R_Adapt_ii_AD' *En_G(adaptG_ii,sim_kk);

        for G_jj=1:sum(Link_Matrix(:,adaptG_ii))
            An_G{adaptG_ii,1}{1,G_jj}=Adapt_phase_ii((G_jj-1)*w_L+1:(G_jj)*w_L,1);
        end  
    end
   
    for comG_ii=1:Num_WFS_source % Step. 2 - Node-based combination 
        for C_jj=1:sum(Link_Matrix(:,comG_ii))        
            Pass_W_n=zeros(w_L,1);
            nei_sum=0;
            for nei_kk=1:sum(Link_Matrix(:,comG_ii)) 
                for nei_kk_ll=1:sum(Link_Matrix(:,Wn_G{comG_ii,1}{2,nei_kk}))
                    if Wn_G{Wn_G{comG_ii,1}{2,nei_kk},1}{2,nei_kk_ll}==Wn_G{comG_ii,1}{2,C_jj}
                       nei_sum=nei_sum+1;
                       Pass_W_n=Pass_W_n+An_G{Wn_G{comG_ii,1}{2,nei_kk},1}{1,nei_kk_ll};
                       % Here, it should be noted that the averaging rule is used in the combination phase of the ADFxLMS algorithm following our previous work: 
                       % Li, T., Lian, S., Zhao, S., Lu, J., and Burnett, I. S. (2023). "Distributed Active Noise Control Based on an Augmented Diffusion FxLMS Algorithm," IEEE/ACM Trans. Audio, Speech, Lang. Process. 31, 1449-1463.
                       % The neighborhood communication restriction is used and disscussed in the Equation (27), which indicates that the combination of non-self control filters only utilizes partial adaptive results.
                       % This restriction will cause the sum of the combination coefficients of non-self control filters based on the Metropolis rule to be less than 1, making the steady-state control filter to approach zero. 
                       % To ensure the noise reduction performance of the ADFxLMS algorithm, we still use the averaging combination rule following our previous work.
                    end
                end 
            end
            Wn_G{comG_ii,1}{1,C_jj}=Pass_W_n/nei_sum;
             
        end
    end 

%--------------------------------------------Iteration of the CFxLMS Algorithm---------------------------------------------%
Yn_C = R_n*Wn_Global_C; % Calculate the anti-noise signal at the error microphones
R_n_CLMS_normal = (R_n_nomal).'; 
En_C(:,sim_kk) = D_n+awgn(Yn_C,SNR_en,'measured'); % Calculate the error signal
Wn_Global_C = Wn_Global_C-mu_CLMS*R_n_CLMS_normal*En_C(:,sim_kk); % Update the global control filter

%-------------------------------------------Iteration of the DCFxLMS Algorithm---------------------------------------------%
Yn_DC=R_n*Wn_Global_DC; % Calculate the anti-noise signal at the error microphones
En_DC(:,sim_kk) = D_n+awgn(Yn_DC,SNR_en,'measured'); % Calculate the error signal
for D_ii=1:Num_WFS_source
Wn_DC(:,D_ii)=Wn_DC(:,D_ii)-(mu_BDLMS/(deltaa+(norm((R_n(D_ii,(D_ii-1)*w_L+1:D_ii*w_L)).',2))^2))*(R_n(D_ii,(D_ii-1)*w_L+1:D_ii*w_L)).'*En_DC(D_ii,sim_kk);
end
for D_ii=1:Num_WFS_source
Wn_Global_DC((D_ii-1)*w_L+1:D_ii*w_L,1)=Wn_DC(:,D_ii); % Update the global control filter
end

%-------------------------------------------Iteration of the MDFxLMS Algorithm---------------------------------------------%
Yn_MD = R_n*Wn_Global_MD; % Calculate the anti-noise signal at the error microphones
En_MD(:,sim_kk) = D_n+awgn(Yn_MD,SNR_en,'measured'); % Calculate the error signal
for M_ii=1:Num_WFS_source % Step. 1 - Node-specific adaptation
An_MD(:,M_ii)=Wn_MD(:,M_ii)-(mu_MDLMS/(deltaa+(norm((R_n(M_ii,(M_ii-1)*w_L+1:M_ii*w_L)).',2))^2))*(R_n(M_ii,(M_ii-1)*w_L+1:M_ii*w_L)).'*En_MD(M_ii,sim_kk);
end

for M_ii=1:Num_WFS_source % Step. 2 - Neighborhood-wide combination
Pass_MD=An_MD*Link_VSR(:,M_ii);%Link_Matrix(:,M_ii)*(1/sum(Link_Matrix(:,M_ii)));
Wn_MD(:,M_ii)=Pass_MD;
end

for M_ii=1:Num_WFS_source % Update the global control filter
Wn_Global_MD((M_ii-1)*w_L+1:M_ii*w_L,1)=Wn_MD(:,M_ii); 
end

%-----------------------------------------Iteration of the MDFxLMS-VSR Algorithm-------------------------------------------%
% Refer to (Chu et al., 2021)-"Chu, Y., Mak, C., Cai, C., and Wu, M. (2021). "A new variable spatial regularized 
% FxLMS algorithm for diffusion active noise control,” Journal of Nanjing University (Natural Science). 57, 683-689."
Yn_MD_VSR=R_n*Wn_Global_MD_VSR; % Calculate the anti-noise signal at the error microphones
En_MD_VSR(:,sim_kk) = D_n+awgn(Yn_MD_VSR,SNR_en,'measured'); % Calculate the error signal

for M_ii=1:Num_WFS_source % Step. 1 - Node-specific adaptation
An_MD_VSR(:,M_ii)=Wn_MD_VSR(:,M_ii)-(mu_MDLMS_VSR/(deltaa+(norm((R_n(M_ii,(M_ii-1)*w_L+1:M_ii*w_L)).',2))^2))*(R_n(M_ii,(M_ii-1)*w_L+1:M_ii*w_L)).'*En_MD_VSR(M_ii,sim_kk);
end

delta_k_n=zeros(Num_WFS_source,1); % Step. 2.1 - Calculate lamda(n+1) according Equation (17) of (Chu et al., 2021)
for Sigma_ii=1:Num_WFS_source
    for Sigma_jj=1:Num_WFS_source
        delta_k_n(Sigma_ii,1)=delta_k_n(Sigma_ii,1)+b_l_k(Sigma_ii,Sigma_jj)*(norm(  (An_MD_VSR(:,Sigma_ii)) - (An_MD_VSR(:,Sigma_jj))  ,2))^2;
    end
end

for Sigma_ii=1:Num_WFS_source % Step. 2.2 - Calculate lamda(n+1) according Equation (17) of (Chu et al., 2021)
    lamda_k_n(Sigma_ii,1)=lamda_k_n(Sigma_ii,1)+beta_k(Sigma_ii,1)*( 0.5*(delta_k_n(Sigma_ii,1)-Sigma_w_k(Sigma_ii,1))-  lamda_k_n(Sigma_ii,1)  );
    alfa_k_n(Sigma_ii,1)=yiks_k(Sigma_ii,1)*(1+kapa_k(Sigma_ii,1)*lamda_k_n(Sigma_ii,1));
end

for Link_ii=1:Num_WFS_source % Step. 3 - Calculate combination coefficients according Equation (13) of (Chu et al., 2021)
    for Link_jj=1:Num_WFS_source
        if Link_ii==Link_jj
           a_l_k_VSR(Link_ii,Link_jj)=1-(1-b_l_k(Link_ii,Link_jj))*alfa_k_n(Link_ii,1);
        else 
           a_l_k_VSR(Link_ii,Link_jj)=b_l_k(Link_ii,Link_jj)*alfa_k_n(Link_ii,1);
        end
    end
end

for M_ii=1:Num_WFS_source % Step. 4 - Neighborhood-wide combination
    Pass_MD_VSR=zeros(w_L,1);
    for N_ii=1:Num_WFS_source
        Pass_MD_VSR=Pass_MD_VSR+a_l_k_VSR(M_ii,N_ii)*An_MD_VSR(:,N_ii);
    end
    Wn_MD_VSR(:,M_ii)=Pass_MD_VSR;
end

for M_ii=1:Num_WFS_source % Update the global control filter
Wn_Global_MD_VSR((M_ii-1)*w_L+1:M_ii*w_L,1)=Wn_MD_VSR(:,M_ii); 
end
end

%% Signal energy calculation
SYSTEM_d_n = sum(d_n.*d_n);                 % The primary noise
GDLMS_E_n_ATCC = sum(En_G_ATCC.*En_G_ATCC); % The ADFxLMS-BC algorithm
CDLMS_E_n = sum(En_C.*En_C);                % The CFxLMS algorithm
DCLMS_E_n = sum(En_DC.*En_DC);              % The DCFxLMS algorithm
MDLMS_E_n = sum(En_MD.*En_MD);              % The MDFxLMS algorithm
GDLMS_E_n = sum(En_G.*En_G);                % The ADFxLMS algorithm
MDLMS_E_n_VSR = sum(En_MD_VSR.*En_MD_VSR);  % The MDFxLMS-VSR algorithm
end 