# Adaptive Multi-sensor Integration for Autonomous vehicles (GNSS/INS/LiDAR/HD Map)

- INS: Pose Predictor
- LiDAR: Mapping with real-time point clouds and offline point clouds 
- HD Map: Pre-built offline point cloud map

This package is under development. Currently, the mapping between the LiDAR and HD Map is provided by Autoware. This is curently robust with: INS/LiDAR/HD Map

-state vector: 
-state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
-control = [acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z]
-measurement=[lx,ly,lz,lqw,lqx,lqy,lqz]


## Spec Recommendation

- Number of CPU cores: 4
- RAM size: 16GB or larger
- Storage size: 30GB in SSD

## Requirements

- ROS jade (Ubuntu 14.04)
- Qt 5.2.1 or higher

### Install dependencies for Ubuntu 14.04 jade

install all the dependency when needed



## How to Build

```
$ cd $HOME
$ mkdir AMSI/src
$ cd AMSI/src
$ git clone https://github.com/weisongwen/AMSI.git
$ catkin_init_workspace
$ cd ..
$ catkin_make
```

## How to Start

```
$ cd $HOME/AMSI/src
$ roslaunch amsi amsi.launch
```

## How to use this for your data

The data is saved in Dropbox. The data for public will be opened soon,


## Research Papers for Reference

1. Wen, Weisong, Guohao Zhang, and Li-Ta Hsu. "Exclusion of GNSS NLOS receptions caused by dynamic objects in heavy traffic urban scenarios using real-time 3D point cloud: An approach without 3D maps." Position, Location and Navigation Symposium (PLANS), 2018 IEEE/ION. IEEE, 2018. (https://ieeexplore.ieee.org/abstract/document/8373377/)
2. under update

## Claim

AMSI is for adaptively integration of multi-sensor information. As some of the code refers to some existing repositories, including [Autoware](https://github.com/CPFL/Autoware). If there is any thing inappropriate, please contact me through 17902061r@connect.polyu.hk (Weisong WEN).


## LICENSE
### BSD License – PolyU

Copyright (c) 2018 [Weisong WEN](https://github.com/weisongwen)

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the <organization> nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
