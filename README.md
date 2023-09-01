# ElC-OIS: Ellipsoidal Clustering for Open-World Instance Segmentation on LiDAR Data

This repository contains the implementation of our paper, which is accepted by 2023 IROS:

> **ElC-OIS: Ellipsoidal Clustering for Open-World Instance Segmentation on LiDAR Data** [[pdf](https://arxiv.org/pdf/2303.04351.pdf)]\
> [Wenbang Deng](https://github.com/dwbzxc),  [Kaihong Huang](https://github.com/mshicom),  Qinghua Yu,  Huimin Lu,  Zhiqiang Zheng,  [Xieyuanli Chen](https://github.com/Chen-Xieyuanli)

If you use our code in your work, please star our repo and cite our paper.

```
@article{deng2023arxiv,
	title={{ElC-OIS: Ellipsoidal Clustering for Open-World Instance Segmentation on LiDAR Data}},
	author={Deng, Wenbang and Huang, Kaihong and Yu, Qinghua and Lu, Huimin and Zheng, Zhiqiang and Chen, Xieyuanli},
	journal={arXiv preprint},
	volume  = {2303.04351},
	year={2023}
}
```

<div align=center>
<img src="./docs/visualization.png"> 
</div>

**Visualization of the segmentation results.** The instances circled with red lines are unknown instances, i.e., instances unlabeled in the training set. The left ones circled with blue lines are known instances, such as car, person, and bicycle.

## Overview

<div align=center>
<img src="./docs/framework.png"> 
</div>

**Overview of our framework.** The framework consists of three main components:
- close-set panoptic segmentation for removing the background and generating raw known instances
- unknown instance clustering for using proposed ellipsoidal clustering to segment unknown instances
- known instance refinement for using proposed diffuse searching to refine raw known instances.

## Installation
The project is tested in Ubuntu 18.04 and Ubuntu 20.04. For installation, run the following command in the root directory of this project:
```
./installation.sh
```

Then, download the close-set panoptic segmentation results of [DS-Net](https://github.com/hongfz16/DS-Net) and [Panoptic-PolarNet](https://github.com/edwardzhou130/Panoptic-PolarNet) in this [link](https://pan.baidu.com/s/1D2d3rO4jOePgKj-LwUMccQ?pwd=ycny), **and unzip it in the root directory of this project.**

## How to run
Run the following command in the root directory:
```
python3 ois_framework.py --dataset_dir /dataset/kitti/dataset/sequences --target_set val --panoptic_segmentation_result DS-net --use_refinement
```
where `--dataset_dir` is the path of [kitti velodyne laser data sequences](https://www.cvlibs.net/datasets/kitti/eval_odometry.php).

## Dataset and benchmark

### Dataset

The close-set panoptic segmentation networks are pretrained on the SemanticKITTI [dataset](http://semantic-kitti.org/). 

### Benchmark

We evaluate our framework on the SemanticKITTI open-world LiDAR instance segmentation [benchmark](https://codalab.lisn.upsaclay.fr/competitions/2183#results). 

## Contact

Please contact us with any questions or suggestions!

Wenbang Deng: wbdeng@nudt.edu.cn and Xieyuanli Chen: xieyuanli.chen@nudt.edu.cn

## License

This project is free software made available under the MIT License. For details see the LICENSE file.

## Acknowledgments
We refer to the following open-source repository:
https://github.com/placeforyiming/ICCVW21-LiDAR-Panoptic-Segmentation-TradiCV-Survey-of-Point-Cloud-Cluster

