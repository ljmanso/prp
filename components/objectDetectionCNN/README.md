```
```
#``` objectDetectionCNN```
This is code for object detection using CNN and image processing. 

## Configuration parameters
As any other component,
``` *objectDetectionCNN* ```
All the configuration parameters are present in:

    etc/config
    etc/caffe_config

You need to install caffe and change required paths in  caffe_config.
    
#Starting the component
Simply run 
```sh run.sh```

The component will load the network and wait for input.
The input is given through remote procedure call. See component:
    testobjectDetectionCNNsimulation
for object detection in simulation environment.
Note: you will need to combine the model file from tar.gz.* in models. You can do that by
cat bvlc_vgg_sim_iter_1200.caffemodel.tar.gz.* | tar xzvf -

    
