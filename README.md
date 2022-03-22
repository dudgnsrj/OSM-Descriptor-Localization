# OSM Descriptor Localization

## Paper
```
@ARTICLE{9716862,
  author={Cho, Younghun and Kim, Giseop and Lee, Sangmin and Ryu, Jee-Hwan},
  journal={IEEE Robotics and Automation Letters}, 
  title={OpenStreetMap-Based LiDAR Global Localization in Urban Environment Without a Prior LiDAR Map}, 
  year={2022},
  volume={7},
  number={2},
  pages={4999-5006},
  doi={10.1109/LRA.2022.3152476}}
```

## How To Use

### Data Preparation
KITTI Raw Data: http://www.cvlibs.net/datasets/kitti/raw_data.php  
KITTI-360: http://www.cvlibs.net/datasets/kitti-360/index.php
```
Sequence: Drive  
00: 2011_10_03_drive_0027  
02: 2011_10_03_drive_0034  
05: 2011_09_30_drive_0018  
06: 2011_09_30_drive_0020  
07: 2011_09_30_drive_0027  
08: 2011_09_30_drive_0028  
09: 2011_09_30_drive_0033  
10: 2011_09_30_drive_0034  

00 includes 07
05 includes 06
09 includes 10

Use {00, 05, 09} OSM descriptors to compare with {07, 06, 10} LiDAR descriptors.
```

### Extract Building Pointcloud
SemanticKITTI Label: http://www.semantic-kitti.org/index.html  
RangeNet++: https://github.com/PRBonn/rangenet_lib
- modified ```src/rangenet_lib/example/infer.cpp``` to apply to multiple files.

### Make Descriptors
Run ```make_descriptor.m```
- please fill ```55 pc_path = ; % Building pointcloud path (folder)``` before run.

### Compare Descriptors
Run ```compare_descriptor.m```
- please fill ```27 pc_pose_path = ; % path to kitti oxts``` before run.
- for KITTI 08, you should comment ```35 files(1:2) = [];```, and uncomment ```36 % files(1:1102) = []; % for sequence 08, use this (and comment out above)```. (KITTI 08 starts from 1100th frame.)

## Contact
```
lucascho@kaist.ac.kr
```

## License
 <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.

### Copyright
- All codes on this page are copyrighted by KAIST and published under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 License. You must attribute the work in the manner specified by the author. You may not use the work for commercial purposes, and you may only distribute the resulting work under the same license if you alter, transform, or create the work.
