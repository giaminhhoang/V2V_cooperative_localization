This repo contains codes used to reproduce the results in:

[1] Shen, Macheng, et al. "Improving localization accuracy in connected vehicle networks using rao-blackwellized particle filters: Theory, simulations, and experiments." IEEE Transactions on Intelligent Transportation Systems (2018). (https://arxiv.org/abs/1702.05792)

[2] Rohani, Mohsen, Denis Gingras, and Dominique Gruyer. "A novel approach for improved vehicular positioning using cooperative map matching and dynamic base station DGPS concept." IEEE Transactions on Intelligent Transportation Systems 17.1 (2016): 230-239.

'main.m' calls the functions that implement three algorithms for cooperative map matching:
1. a Rao-blackwellized particle filter;
2. a kalman-smoothed static method;
3. a static method. (proposed by Rohani et al.)

Installation: The main function needs Matlab SatNav toolbox as a support.
See http://www.navtechgps.com/satnav_toolbox/
Include the SatNav toolbox path before running.

Acknowledgement: Some of the m.file for post-processing such as 'plotcov2d', 'plotmarker' and 'plotSamples'
are taken from Professor Ryan Eustice's course pack of NAME/EECS 568 Mobile Robotics @ University of Michigan, Ann Arbor.

'obs1','obs2',...'obs4' and 'nav1',...'nav4' are raw observables and navigation data collected from four u-blox EVK-6T GNSS receivers,
see the experiment described in [1] for detail.
