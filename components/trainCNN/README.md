```
```
#trainCNN
This is code for training CNN for object detection using caffe. 
It could be used to train the rendered images from RenderOSG component.
By default this uses VGG16 model for training.


## Configuration parameters
All the configuration paprameters are stored in etc/caffe_config.
You will need caffemodel.
Download VGG_ILSVRC_16_layers.caffemodel and put it in model folder using:
wget -c http://www.robots.ox.ac.uk/~vgg/software/very_deep/caffe/VGG_ILSVRC_16_layers.caffemodel

    
## Starting the component
To run the component use:
>sh run.sh
A demo image dataset for 3 classes has been provided.

## Directory structure
use "Data" to store images, train list and lmdb files.
Note: we are using all data for training. You can add a line in specificworker.cpp to generate test.lmbd.

use "Data" to store images, train list and lmdb files.
use "model" to store prototxt, default weights and solver.
use "snapshot" to store trained weight files, that could be directly used in objectdetectionCNN component.
Note: please modify deploy.prototxt accordingly.

## Other remarks
if you get following errors try reducing batch size in you model file (mytrain_val.prototxt)
Check failed: error == cudaSuccess (9 vs. 0)  invalid configuration argument
Check failed: error == cudaSuccess (2 vs. 0)  out of memory
