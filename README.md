# NITCAD-dataset

As part of our major project, we have worked on to develop a dataset for autonomous vehicles in India (named it as NITCAD i.e National Insititute of Technology Calicut Autonomous Driving). 

Please refer to [NITCAD dataset paper](https://doi.org/10.1016/j.procs.2020.04.022) to understand more about the dataset. Alternatively you can read [this Medium Blog](https://namburisrinath.medium.com/nitcad-an-object-detection-classification-and-stereo-vision-dataset-for-autonomous-navigation-f28d3fe5b7d9) for basic introduction to dataset. Fill [this Google form link](https://docs.google.com/forms/d/e/1FAIpQLScv2QXTYy3jwiAlqC9ro-lR_4UhUaWtBDNYQ4jWLZ3eUv3nSA/viewform) (*also attached in blogpost*) to get access to this dataset. 

If this work helps in your research, please consider to cite as

    @article{srinath2020nitcad,
      title={NITCAD-Developing an object detection, classification and stereo vision dataset for autonomous navigation in Indian roads},
      author={Srinath, Namburi GNVV Satya Sai and Joseph, Athul Zac and Umamaheswaran, S and Priyanka, Ch Lakshmi and Nair, Malavika and Sankaran, Praveen},
      journal={Procedia Computer Science},
      volume={171},
      pages={207--216},
      year={2020},
      publisher={Elsevier}
    }

Also you can refer to [Speed estimation using Stereo Vision images](https://ieeexplore.ieee.org/abstract/document/9031876?casa_token=qeCiQNa9m50AAAAA:lOe4ogBfc866e3gPs2s6yesqeHqJ22WElxCQxdl_luLtbeTrgb_eluUFsmMrr8040A_S8U1Lof4y) for the speed estimation by using SIFT (Scale Invariant Feature Transform), YOLO and MC-CNN.

If this work helps in your research, please consider to cite as

    @inproceedings{umamaheswaran2019stereo,
      title={Stereo Vision Based Speed Estimation for Autonomous Driving},
      author={Umamaheswaran, S and Nair, Malavika and Joseph, Athul Zac and Srinath, Namburi GNVV Satya Sai and Priyanka, Ch Lakshmi and Sankaran, Praveen},
      booktitle={2019 International Conference on Information Technology (ICIT)},
      pages={201--205},
      year={2019},
      organization={IEEE}
    }

In addition to that, we worked on KITTI dataset and here are few algorithms that we worked

## KITTI LiDAR dataset
1. Gaussian Mixture Model
2. Fast Segmentation
3. Connected Component Labelling

## Object detection 
1. Faster R-CNN
2. YOLO

## Depth estimation by Stereo Vision based approach
1. MC-CNN (Matching Cost CNN)
2. Inbuild functions from OpenCV

Monocular Visual Odometry (Kanade–Lucas–Tomasi feature tracker)

Object recognition by DenseNet, Inceptionv3, MobileNet, NASNet, VGG16 and Xception (Keras implementation)

## License
[![CC BY-NC-ND 4.0](https://i.creativecommons.org/l/by-nc-nd/4.0/88x31.png)](https://creativecommons.org/licenses/by-nc-nd/4.0/)

NITCAD - An object detection, classification and stereo vision dataset for autonomous navigation in Indian roads by Namburi GNVV Satya Sai Srinath, Athul Zac Joseph, S Umamaheswaran, Ch. Lakshmi Priyanka, Malavika Nair M, Praveen Sankaran is licensed under a [Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License](https://creativecommons.org/licenses/by-nc-nd/4.0/). Based on a work at https://www.sciencedirect.com/science/article/pii/S187705092030987X?via%3Dihub%3C/a%3E

I would like to thank my project teammates [Athul Zac Joseph](https://www.linkedin.com/in/athul-zac-joseph-450564129/), [Ch. Lakshmi Priyanka](https://www.linkedin.com/in/lakshmi-priyanka/), [Malavika Nair M](https://www.linkedin.com/in/malavika-nair-m/), [S Umamaheswaran](https://www.linkedin.com/in/umamaheswaran-s/) and our guide [Dr. Praveen Sankaran](http://www.nitc.ac.in/index.php/?url=users/view/320/10/3) who helped at various stages during the project.

**Note: This repository is in progress and additional links/data will be added.**

**Pending tasks**

- [ ] Add odometry videos
- [ ] Add codes related to deep learning (object classification)
