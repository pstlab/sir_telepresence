# SI-Robotics Cloud

This folder contains the code for the SI-Robotics cloud. In particular, this folder contains the code for:

 - The **REST API**, for providing access to the information of the house, like its devices and its associated users;
 - The **MQTT Broker**, for the exchange of the information from inside to outside, and vice versa, of the house;
 - The **Web Server**, for providing a web-based graphical interface to the different users.

### Compiling the SI-Robotics Cloud

The SI-Robotics Cloud is a [Maven](https://maven.apache.org) project. It is therefore mandatory, in order to compile it, to have a [Java compiler](https://www.oracle.com/it/java/technologies/javase-downloads.html) (version 1.8 or higher) and Maven, properly configured.

## The REST API

The REST API is intended to provide information of the house as well as to allow admin users, through the web interface, to configure the house, associating it devices and users.

The next things to do are:
 - [ ] Create a REST API for managing the users, creating new accounts, editing and deleting them.
 - [ ] Create a REST API for assigning roles to the users.
 - [ ] Create a REST API for creating and editing device types.
 - [ ] Create a REST API for managing the houses, adding new devices, editing and deleting them.

## The MQTT Broker

The MQTT Broker, for the moment, is intended to work on a local network. In particular, the IP address is the address of the machine on which the SI-Robotics Cloud is running (i.e., `localhost`), while the communication port is set to `1883`.

### MQTT Topics

The topics used for the publication of data by the sensors, to which, therefore, it is necessary to subscribe to know the new values in real time, have the following structure:

```
<house-id>/<device-id>
```

Specifically, `<house-id>` is the house's identifier. Similarly, `<device-id>` is the sensor's identifier.

The next things to do are:
 - [ ] Create a web interface for mimicking sensors, so as to allow to produce fake data.

## The Database Structure

A preliminary entity-relation diagram of the database is shown in the following figure.

![ER Diagram](docs/figs/db.png?raw=true "ER Diagram")
