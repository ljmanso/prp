
# testobjectDetectionCNN
This component reads an RGB image from argument and passes it to objectDetectionCNN component for detection and displays the result in form of bounding boxes and labels on the image.

## Configuration parameters
As any other component,
``` *testobjectDetectionCNN* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file in /etc. 
    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <testobjectDetectionCNN 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component.
This component requires objectDetectionCNN. So first run :
```../objectDetectionCNN/bin/objectDetectionCNN --Ice.Config=config```
In another terminal run:
```bin/objectDetectionCNN --Ice.Config=config --src=<source image>```
Note to match the port of objectDetectionCNNProxy in config file with objectDetectionCNN's config file.

