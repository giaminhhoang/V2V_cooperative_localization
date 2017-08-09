
Need SatNav toolbox as a support

See http://www.navtechgps.com/satnav_toolbox/

Include the SatNav toolbox path before running.

'main_CMM.m' calls the functions that implement RBPF, smoothed static method and static method for CMM.

For detial, see Shen et al. [1], [2]

[1] Shen, Macheng, et al. "Improving localization accuracy in connected vehicle networks using rao-blackwellized particle filters: Theory, simulations, and experiments." arXiv preprint arXiv:1702.05792 (2017).

[2] Rohani, Mohsen, Denis Gingras, and Dominique Gruyer. "A novel approach for improved vehicular positioning using cooperative map matching and dynamic base station DGPS concept." IEEE Transactions on Intelligent Transportation Systems 17.1 (2016): 230-239.

Some of the m.file doing elementary work such as 'plotcov2d', 'plotmarker' and 'plotSamples'

are taken from Professor Ryan Eustice's course pack of NAME/EECS 568 Mobile Robotics @ University of Michigan, Ann Arbor.

'obs1','obs2',...'obs4' and 'nav1',...'nav4' are raw observables and navigation data collected from four u-blox EVK-6T GNSS receivers,

see the experiment described in [1] for detail.# GPS_project
