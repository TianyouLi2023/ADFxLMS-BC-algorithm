# ADFxLMS-BC-algorithm
This website is supplementary material for the article "Tianyou Li, Li Rao, Sipei Zhao, Jing Lu and Ian S. Burnett; An Improved Augmented Diffusion Algorithm with Bidirectional Communication for a Distributed Active Noise Control System; J. Acoust. Soc. Am., 2023". This website provides the MATLAB source codes for all simulations. 

(c) 2023 by Tianyou Li (tianyou.li@smail.nju.edu.cn), Li Rao (li.rao@smail.nju.edu.cn), Sipei Zhao (sipei.zhao@uts.edu.au), Hongji Duan (duan@smail.nju.edu.cn), Jing Lu (lujing@nju.edu.cn) and Ian S. Burnett (Ian S. Burnett: ian.burnett2@rmit.edu.au)

%------------------------------------------------------------------------------------------------%

The folder "Figs_5_6" provides the MATLAB source codes of the Figures 5 and 6, which corresponds to the free-field simulation of the above article.

"Main1_Monte_Carlo.m" is the main function of the whole Simulation experiments, which provides the Moving-average and Monte Carlo simulation environment.

"Main2_JASA_Single_Simulation.m" can execute a noise reduction simulation experiment.

"Calculate_user_coef.m" is used to calculate the user coefficients of the MDFxLMS-VSR algorithm. The algorithm details of the MDFxLMS-VSR algorithm can be found in reference "Chu, Y., Mak, C., Cai, C., and Wu, M. (2021). “A new variable spatial regularized FxLMS algorithm for diffusion active noise control,” Journal of Nanjing University (Natural Science). 57, 683-689." 

%------------------------------------------------------------------------------------------------%

The folder "Figs_8_9" provides the MATLAB source codes of the Figures 8 and 9, which corresponds to the reverberation room simulation of the above article.

"Main1_Monte_Carlo.m" is the main function of the whole Simulation experiments, which provides the Moving-average and Monte Carlo simulation environment.

"Main2_JASA_Single_Simulation.m" can execute a noise reduction simulation experiment.

"Calculate_user_coef.m" is used to calculate the user coefficients of the MDFxLMS-VSR algorithm. The algorithm details of the MDFxLMS-VSR algorithm can be found in reference "Chu, Y., Mak, C., Cai, C., and Wu, M. (2021). “A new variable spatial regularized FxLMS algorithm for diffusion active noise control,” Journal of Nanjing University (Natural Science). 57, 683-689."

"Primary_path_DSP" denotes the acoustic paths between the primary noise source and all error microphones. 

"Secondary_path_k" denotes the acoustic paths between the kth secondary loudspeaker and all error microphones.

%------------------------------------------------------------------------------------------------%

P.s. Our works on "Distributed Active Noise Control System Based on the Augmented Diffusion Strategy" include:

(1) Tianyou Li, Siyuan Lian, Sipei Zhao, Jing Lu and Ian S. Burnett, "Distributed Active Noise Control Based on an Augmented Diffusion FxLMS Algorithm," IEEE/ACM Transactions on Audio, Speech, and Language Processing, vol. 31, pp. 1449-1463, 2023. DOI: https://doi.org/10.1109/TASLP.2023.3261742

(2) Tianyou Li, Li Rao, Sipei Zhao, Jing Lu and Ian S. Burnett, "An Improved Augmented Diffusion Algorithm with Bidirectional Communication for a Distributed Active Noise Control System," The Journal of the Acoustic Society of America, vol. 154, pp. 3568-3579, 2023. DOI: https://doi.org/10.1121/10.0022573

(3) Tianyou Li, Sipei Zhao, Kai Chen and Jing Lu, "A diffusion filtered-x affine projection algorithm for distributed active noise control," Inter-noise 2023, pp. 3050-3057(8). DOI: https://doi.org/10.3397/IN_2023_0441

Can you give me a Star (on the top of the page) if that our works are useful for your researches? ('◡')
