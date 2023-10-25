# ADFxLMS-BC-algorithm
This website is supplementary material for the article "Tianyou Li, Hongji Duan, Sipei Zhao, Jing Lu and Ian S. Burnett; An Improved Augmented Diffusion Algorithm with Bidirectional Communication for a Distributed Active Noise Control System; J. Acoust. Soc. Am. (Under Review), 2023.10". This website provides the MATLAB source codes for all simulations. 

(c) 2023 by Tianyou Li, Hongji Duan, Sipei Zhao, Jing Lu and Ian S. Burnett

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

P.s. Our works on "Distributed Active Noise Control System Based on the Augmented Diffusion Strategy" include:

(1) Tianyou Li, Siyuan Lian, Sipei Zhao, Jing Lu and Ian S. Burnett, "Distributed Active Noise Control Based on an Augmented Diffusion FxLMS Algorithm," IEEE/ACM Trans. Audio, Speech, Lang. Process., vol. 31, pp. 1449-1463, 2023.

(2) Tianyou Li, Sipei Zhao, Kai Chen and Jing Lu, "A diffusion filtered-x affine projection algorithm for distributed active noise control," Inter-noise 2023, 2023-3-3-8, 2023.

(3) Tianyou Li, Hongji Duan, Sipei Zhao, Jing Lu and Ian S. Burnett, "An Improved Augmented Diffusion Algorithm with Bidirectional Communication for a Distributed Active Noise Control System," The Journal of the Acoustic Society of America (Under Review), 2023.
