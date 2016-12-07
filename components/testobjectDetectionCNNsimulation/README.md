#testODCNNsimulation
This component provides a platform where you can load an environment file in rcis simulator. Move robot around and detect the objects captured through robot's camera. 


## Configuration parameters
As any other component,
``` *testODCNNsimulation* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file in /etc. 
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <testODCNNsimulation 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component.
This component requires a simulation environment.
```rcis ../../experimentFiles/simulation/tabletop.xml  ```
You should see environment with a robot and objects paced on tables.
Also objectDetectionCNN is required. In another terminal run:

    ../objectDetectionCNN/bin/objectDetectionCNN --Ice.Config=config

Then you can run this component by: 

        bin/testODCNNsimulation --Ice.Config=config
You should see a button panel. Click on appropriate button to move robot around. Once you see objects on table you can detect them by pressing object detection button.
