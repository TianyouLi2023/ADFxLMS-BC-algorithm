%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This work is supplementary material for the article                        %
%                                                                            %
% Tianyou Li, Hongji Duan, Sipei Zhao, Jing Lu and Ian S. Burnett            %
%   An Improved Augmented Diffusion Algorithm with Bidirectional             %
%   Communication for a Distributed Active Noise Control System              %
% The Journal of the Acoustic Society of America (Under Review), 2023.10     %
%                                                                            %
%                                                                            %
%(c) 2023 by Tianyou Li, Hongji Duan, Sipei Zhao, Jing Lu and Ian S. Burnett %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%"Main1_Monte_Carlo.m" is the main function of the whole Simulation experiments, which provides the Moving-average and Monte Carlo simulation environment. 
clc;clearvars;close all;
rng(0);

Men_Num = 100;      % The number of the Monte Carlo simulation
window_length = 20; % Moving-average window length

Ts = 40;    % The time length of Reverberation-room simulations (second)
Fs = 4000; % Sampling Frequency (Hz)

% The global results of the Monte Carlo simulation
BD_normal_en = zeros(1,Ts*Fs);     % The global error of the ADFxLMS-BC algorithm
BD_normal_en_NBC = zeros(1,Ts*Fs); % The global error of the ADFxLMS algorithm
CD_normal_en = zeros(1,Ts*Fs);     % The global error of the CFxLMS algorithm
DC_normal_en = zeros(1,Ts*Fs);     % The global error of the DCFxLMS algorithm
MD_normal_en = zeros(1,Ts*Fs);     % The global error of the MDFxLMS algorithm
MD_VSR_normal_en = zeros(1,Ts*Fs); % The global error of the MDFxLMS-VSR algorithm
Desire_n = zeros(1,Ts*Fs);         % The global primary noise 

for Men_ii = 1:Men_Num
    Men_ii % The current number of the completed Monte Carlo simulations
    
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
    % (5) SYSTEM_d_n denotes the global primary noise. 
    
    [w_L,Num_WFS_source,Wn_Global_GD_ATCC,Wn_Global_GD,Wn_Global_C,Wn_Global_DC,Wn_Global_MD,Wn_Global_MD_VSR,CDLMS_E_n,DCLMS_E_n,MDLMS_E_n,GDLMS_E_n,MDLMS_E_n_VSR,GDLMS_E_n_ATCC,SYSTEM_d_n]=...
    Main2_JASA_Single_Simulation(1);

    BD_normal_en=BD_normal_en+GDLMS_E_n_ATCC;
    BD_normal_en_NBC=BD_normal_en_NBC+GDLMS_E_n;
    CD_normal_en=CD_normal_en+CDLMS_E_n;
    DC_normal_en=DC_normal_en+DCLMS_E_n;
    MD_normal_en=MD_normal_en+MDLMS_E_n;
    MD_VSR_normal_en=MD_VSR_normal_en+MDLMS_E_n_VSR;
    Desire_n=Desire_n+SYSTEM_d_n;
end

% The global results of the Monte Carlo simulation
BD_normal_en=BD_normal_en/Men_Num;
BD_normal_en_NBC=BD_normal_en_NBC/Men_Num;
CD_normal_en=CD_normal_en/Men_Num;
DC_normal_en=DC_normal_en/Men_Num;
MD_normal_en=MD_normal_en/Men_Num;
MD_VSR_normal_en=MD_VSR_normal_en/Men_Num;
Desire_n=Desire_n/Men_Num;

% The calculation of the moving-average result after the Monte Carlo simulation
% The CFxLMS algorithm
for wwl=1:length(CD_normal_en)-window_length+1
En_CDLMS(wwl,1)=10*log10((sum(CD_normal_en(1,wwl:wwl+window_length-1))/window_length)...
                /(sum(Desire_n(1,wwl:wwl+window_length-1))/window_length));
if En_CDLMS(wwl,1)>=0 || isnan(En_CDLMS(wwl,1))
En_CDLMS(wwl,1)=0;
end
end

% The DCFxLMS algorithm
for wwl=1:length(DC_normal_en)-window_length+1
En_DCLMS(wwl,1)=10*log10((sum(DC_normal_en(1,wwl:wwl+window_length-1))/window_length)...
                /(sum(Desire_n(1,wwl:wwl+window_length-1))/window_length));
if En_DCLMS(wwl,1)>=0 || isnan(En_DCLMS(wwl,1))
En_DCLMS(wwl,1)=0;
end
end

% The MDFxLMS algorithm
for wwl=1:length(MD_normal_en)-window_length+1
En_MDLMS(wwl,1)=10*log10((sum(MD_normal_en(1,wwl:wwl+window_length-1))/window_length)...
                /(sum(Desire_n(1,wwl:wwl+window_length-1))/window_length));
if En_MDLMS(wwl,1)>=0 || isnan(En_MDLMS(wwl,1))
En_MDLMS(wwl,1)=0;
end
end

% The MDFxLMS-VSR algorithm
for wwl=1:length(MD_VSR_normal_en)-window_length+1
En_MDLMS_VSR(wwl,1)=10*log10((sum(MD_VSR_normal_en(1,wwl:wwl+window_length-1))/window_length)...
                /(sum(Desire_n(1,wwl:wwl+window_length-1))/window_length));
if En_MDLMS_VSR(wwl,1)>=0 || isnan(En_MDLMS_VSR(wwl,1))
En_MDLMS_VSR(wwl,1)=0;
end
end

% The ADFxLMS-BC algorithm
for wwl=1:length(BD_normal_en)-window_length+1
En_BDLMS(wwl,1)=10*log10((sum(BD_normal_en(1,wwl:wwl+window_length-1))/window_length)...
                /(sum(Desire_n(1,wwl:wwl+window_length-1))/window_length));
if En_BDLMS(wwl,1)>=0 || isnan(En_BDLMS(wwl,1))
En_BDLMS(wwl,1)=0;
end
end

% The ADFxLMS algorithm
for wwl=1:length(BD_normal_en_NBC)-window_length+1
En_BDLMS_NBC(wwl,1)=10*log10((sum(BD_normal_en_NBC(1,wwl:wwl+window_length-1))/window_length)...
                /(sum(Desire_n(1,wwl:wwl+window_length-1))/window_length));
if En_BDLMS_NBC(wwl,1)>=0 || isnan(En_BDLMS_NBC(wwl,1))
En_BDLMS_NBC(wwl,1)=0;
end
end

% Fig. 8. The noise reduction performance of broadband noise using different algorithms in a reverberation room. 
En_CDLMS(1,1)=0;En_DCLMS(1,1)=0;En_MDLMS(1,1)=0;En_BDLMS(1,1)=0;En_MDLMS_VSR(1,1)=0;En_BDLMS_NBC(1,1)=0;
figure;tt=0:1/Fs:Ts-window_length/Fs;ss=19998;
h1=plot(tt,En_CDLMS,'Color',[255 0 0]/255,'LineWidth',1,'Marker','*','Markersize',8,'MarkerIndices',1:ss:length(tt));hold on;
h2=plot(tt,En_DCLMS,'Color',[0 0 0]/255,'LineWidth',1.3,'Marker','x','Markersize',8,'MarkerIndices',1:ss:length(tt));hold on;
h3=plot(tt,En_MDLMS,'Color',[255 0 255]/255,'LineWidth',1,'Marker','+','Markersize',8,'MarkerIndices',1:ss:length(tt));hold on;
h4=plot(tt,En_MDLMS_VSR,'Color',[0 180 20]/255,'LineWidth',1,'Marker','s','Markersize',8,'MarkerIndices',1:ss:length(tt));hold on;
h5=plot(tt,En_BDLMS_NBC,'Color',[155 20 100]/255,'LineWidth',1,'Marker','s','Markersize',8,'MarkerIndices',1:ss:length(tt));hold on;
h6=plot(tt,En_BDLMS,'Color',[0 0 255]/255,'LineWidth',1,'Marker','o','Markersize',8,'MarkerIndices',1:ss:length(tt));hold on;
axis([0 Ts -20 1]);
set(gca,'FontSize',16,'FontName','Times New Roman');
xlabel('Iteration Time / s','FontName','Times New Roman','FontSize',16);
ylabel('Normalized Residual Noise / dB','FontName','Times New Roman','FontSize',16);grid on;

lgd1=legend([h1,h2],'CFxLMS','DCFxLMS','orientation','horizontal','location','north');
set(lgd1,'FontSize',16,'FontName','Times New Roman');
ah1=axes('position',get(gca,'position'),'visible','off');
lgd2=legend(ah1,[h3,h4],'MDFxLMS','MDFxLMS-VSR','orientation','horizontal','location','north');
set(lgd2,'FontSize',16,'FontName','Times New Roman');
ah2=axes('position',get(gca,'position'),'visible','off');
lgd3=legend(ah2,[h5,h6],'ADFxLMS','ADFxLMS-BC','orientation','horizontal','location','north');
set(lgd3,'FontSize',16,'FontName','Times New Roman');
ah3=axes('position',get(gca,'position'),'visible','off');

%Fig. 9. The converged global control filters of (a) CFxLMS, (b) DCFxLMS (c) MDFxLMS, (d) MDFxLMS-VSR, (e) ADFxLMS and (f) ADFxLMS-BC algorithms in a reverberation room.
figure;plot(Wn_Global_C,'Color',[255 0 0]/255);legend('CFxLMS','FontSize',16,'FontName','Times New Roman','Location','southwest');grid on;axis([0 22000 -0.1 0.05]);
set(gca,'FontSize',16,'FontName','Times New Roman');
xlabel('Tap','FontName','Times New Roman','FontSize',16);ylabel('Global Weight Vector','FontName','Times New Roman','FontSize',16);
figure;plot(Wn_Global_DC,'Color',[0 0 0]/255);legend('DCFxLMS','FontSize',16,'FontName','Times New Roman','Location','southwest');grid on;axis([0 22000 -1.5e20 1.5e20]);
set(gca,'FontSize',16,'FontName','Times New Roman');
xlabel('Tap','FontName','Times New Roman','FontSize',16);ylabel('Global Weight Vector','FontName','Times New Roman','FontSize',16);
figure;plot(Wn_Global_MD,'Color',[255 0 255]/255);legend('MDFxLMS','FontSize',16,'FontName','Times New Roman','Location','southwest');grid on;axis([0 22000 -0.1 0.05]);
set(gca,'FontSize',16,'FontName','Times New Roman');
xlabel('Tap','FontName','Times New Roman','FontSize',16);ylabel('Global Weight Vector','FontName','Times New Roman','FontSize',16);
figure;plot(Wn_Global_MD_VSR,'Color',[0 180 20]/255);legend('MDFxLMS-VSR','FontSize',16,'FontName','Times New Roman','Location','southwest');grid on;axis([0 22000 -0.1 0.05]);
set(gca,'FontSize',16,'FontName','Times New Roman');
xlabel('Tap','FontName','Times New Roman','FontSize',16);ylabel('Global Weight Vector','FontName','Times New Roman','FontSize',16);
figure;plot(Wn_Global_GD,'Color',[155 20 100]/255);legend('ADFxLMS','FontSize',16,'FontName','Times New Roman','Location','southwest');grid on;axis([0 22000 -0.1 0.05]);
set(gca,'FontSize',16,'FontName','Times New Roman');
xlabel('Tap','FontName','Times New Roman','FontSize',16);ylabel('Global Weight Vector','FontName','Times New Roman','FontSize',16);
figure;plot(Wn_Global_GD_ATCC,'Color',[0 0 255]/255);legend('ADFxLMS-BC','FontSize',16,'FontName','Times New Roman','Location','southwest');grid on;axis([0 22000 -0.1 0.05]);
set(gca,'FontSize',16,'FontName','Times New Roman');
xlabel('Tap','FontName','Times New Roman','FontSize',16);ylabel('Global Weight Vector','FontName','Times New Roman','FontSize',16);
