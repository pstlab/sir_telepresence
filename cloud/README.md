# SI-Robotics Cloud

This folder contains the code for the SI-Robotics cloud. In particular, this folder contains the code for:

 - the **MQTT broker**, for the exchange of the information from inside to outside, and vice versa, of the house;
 - the **REST API**, for providing access to the information of the house;
 - the **Web server**, for providing a web-based graphical interface to the different users.

## Database Structure

A preliminary entity-relation diagram of the database is shown in the following figure.

![ER Diagram](docs/figs/db.png?raw=true "ER Diagram")

## Compiling the SI-Robotics Cloud

The SI-Robotics Cloud is a [Maven](https://maven.apache.org) project. It is therefore mandatory, in order to compile it, to have a [Java compiler](https://www.oracle.com/it/java/technologies/javase-downloads.html) (version 1.8 or higher) and Maven, properly configured.
