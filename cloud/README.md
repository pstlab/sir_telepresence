# SI-Robotics Cloud

This folder contains the code for the SI-Robotics cloud. In particular, this folder contains the code for:

 - The **REST API**, for providing access to the information of the house, like its devices and its associated users;
 - The **MQTT Broker**, for the exchange of the information from inside to outside, and vice versa, of the house;
 - The **Web Server**, for providing a web-based graphical interface to the different users.

### Compiling the SI-Robotics Cloud

The SI-Robotics Cloud is a [Maven](https://maven.apache.org) project. It is therefore mandatory, in order to compile it, to have a [Java compiler](https://www.oracle.com/it/java/technologies/javase-downloads.html) (version 1.8 or higher) and Maven, properly configured. The compilation is done through the command line execution of the following command:

```
mvn compile
```

### Running the SI-Robotics Cloud

The easiest way to run the SI-Robotics Cloud is, again, through Maven. In particular, it is sufficient to run the following command to start all the SI-Robotics Cloud services:

```
mvn exec:java
```

## The REST API

The REST API is intended to provide information of the house as well as to allow admin users, through the web interface, to configure the house, associating it devices and users.

##### Next things to do
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

#### Natural Language Interaction

As for the natural language interaction that the robot can have with people inside the house, we need two channels for each house: an `in` channel, from the person to the system, contains the text uttered by the user, and an `out` channel, from the system to the person, contains the text to be spoken by the robot. Considering that the robot is, to all intents and purposes, a *device*, these channels translate into the following MQTT topics:

```
<house-id>/<device-id>/nlp/in/<utterance>
```

```
<house-id>/<device-id>/nlp/out/<utterance>
```

Moreover, to allow the customization of the interaction, the language generation module needs some customization parameters. A channel is therefore necessary to update these parameters. The format of the expected commands is, simply, an array of name-value pairs (i.e., `<name-value>+`) that identify those parameters whose value has changed and indicate their new value.

An example of such a message can be the update of the emotions perceived from the user's face, such as

```json
[{"fearful":0.3}, {"angry": 0.15}, {"happy": 0.55}]
```

#### Planning and Plan Execution

Similar to natural language interaction, we also need two channels in the case of the commands for planning and plan executions: an `in` channel, from the robot to the cloud, and an `out` channel, from the cloud to the robot. Channel names, in particular, are viewed from the planner's perspective. In other words, the commands produced by the planner to be executed on the robotic platform are written on the `out` channel. On the contrary, commands produced by the robotic platform for the planner are written on the `in` channel.

Over time, the plan executor produces the commands that the robotic platform must execute. These commands are characterized by an identifier, a predicate symbol, and a set of parameters (whose cardinality depends on the predicate) represented by name-value pairs. In general, multiple commands can arrive at the same time. In conclusion, a message for the robotic platform produced by the planner will have the following structure:

```
[<id> <predicate-name> <name-value>*]+
```

An example of a message for the robotic platform could be, for example, the navigation command:

```json
[{"id":42, "predicate":"goto", "parameters":[{"room":"r0"}, {"start": 15}, {"end": 30}]}]
```

These commands will hence be published by the planner on the MQTT topic:

```
<house-id>/<device-id>/planner/out
```

Finally, as regards the commands produced by the robotic platform for the planner, we have four types of commands:

The **plan** commands, aiming at generating a plan and executing it, contain the text of the planning problem, are written on the following topic:

```
<house-id>/<device-id>/planner/in/plan
```

The **cant-start-yet** commands, aiming at delaying the starting time of some commands, contain an array of the ids which can't start yet, are written on the following topic:

```
<house-id>/<device-id>/planner/in/cant-start-yet
```

The **cant-finish-yet** commands, aiming at delaying the ending time of some commands, contain an array of the ids which can't finish yet, are written on the following topic:

```
<house-id>/<device-id>/planner/in/cant-finish-yet
```

The **failure** commands, aiming at notifying the planner that the execution of some commands lead to some kind of failure (hence, the planner should produce an alternative plan), contain an array of the ids lead to the failure, are written on the following topic:

```
<house-id>/<device-id>/planner/in/failure
```

##### Next things to do
 - [ ] Connect the planner to the proper MQTT topic.
 - [ ] Create a web interface for mimicking sensors, so as to allow to produce fake data.

## The Database Structure

A preliminary entity-relation diagram of the database is shown in the following figure.

![ER Diagram](docs/figs/db.png?raw=true "ER Diagram")
