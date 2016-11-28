```
```
#
``` RenderOSG
```
This component takes imput as CAD models and renders them in 360 degree rotations poses.
It produces RGB and depth images as output in specified folders.
Multiple files could be given as input when specified in configurable modelfile.
The congiguration file for giving the input model file and output directiories is fixed as etc\renderer_config.
Other essential parameters for rendering like output image size, number of bins etc. are also specified in etc\renderer_config.



## Dependencies
This module requires OSG and boost.
These libraries are precompiled in lib folder.

    
## Starting the component
To start the component run:
>sh run.sh
