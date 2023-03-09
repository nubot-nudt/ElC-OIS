# ElC-OIS: Ellipsoidal Clustering for Open-World Instance Segmentation on LiDAR Data

This repository contains the implementation of our paper:

> **ElC-OIS: Ellipsoidal Clustering for Open-World Instance Segmentation on LiDAR Data** [[pdf](https://arxiv.org/abs/2303.04351)]\
> [Wenbang Deng](https://github.com/dwbzxc),  [Kaihong Huang](https://github.com/mshicom),  Qinghua Yu,  Huimin Lu,  Zhiqiang Zheng,  [Xieyuanli Chen](https://github.com/Chen-Xieyuanli)   
> If you use our code in your work, please star our repo and cite our paper.

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

- *The instances circled with red lines are unknown instances, i.e., instances unlabeled in the training set. The left ones circled with blue lines are known instances, such as car, person, and bicycle.*

## Overview

![framework](./docs/framework.png)

*Overview of our framework. The framework consists of three main components:*
- *close-set panoptic segmentation for removing the background and generating raw known instances*
- *unknown instance clustering for using proposed ellipsoidal clustering to segment unknown instances*
- *known instance refinement for using proposed diffuse searching to refine raw known instances.*

## Data

SemanticKITTI: Download SemanticKITTI dataset from the official [website](http://semantic-kitti.org/). 

## Code

The code and usage details will be available soon.

## Contact

Any question or suggestions are welcome!

Wenbang Deng: wbdeng@nudt.edu.cn and Xieyuanli Chen: xieyuanli.chen@nudt.edu.cn

## License

This project is free software made available under the MIT License. For details see the LICENSE file.
