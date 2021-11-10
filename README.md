# SI-Robotics Telepresence

This repository aims to collect the code of all the modules related to the telepresence aspects of the the SI-Robotics project.



**Old qui readme**

Compile oRatio in the main directory and compile it using the make install in order to install the libraries in /usr/local/lib
Enter in the cloud and compile with mvn package
To run the cloud enter in the directory  sir_telepresence/cloud/target and use the command java -Djava.library.path=/usr/local/lib -jar telepresence-1.0.jar
When pulling, remember to delete the database at sir_telepresence/cloud/target/data and derby.log
Then you should download your config file for mqtt_bridge by connecting to http://localhost:7000/ and by adding houses, sensors, robots 

####################################
**for RASA**
```
cd sir_telepresence/nlu  #every time to cancel cloud directory
rasa train   
```

One shell
```
stack exec duckling-example-exe
```
One shell
```
sir_telepresence/nlu$ rasa shell per testare
sir_telepresence/nlu$ rasa run --enable-api con cloud
```
One shell
```
sir_telepresence/nlu$ rasa run actions
```
####################################


**New version November 2021**
```
git clone --recurse-submodules https://github.com/pstlab/sir_telepresence
git checkout v1.1
git submodule update --init --recursive
```
