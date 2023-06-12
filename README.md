# SI-Robotics Telepresence

This repository aims to collect the code of all the modules related to the telepresence aspects of the the SI-Robotics project. The project is developed by the [PSTLab](https://www.istc.cnr.it/it/group/pst) of the [Institute of Cognitive Sciences and Technologies](https://www.istc.cnr.it) of the [National Research Council of Italy](https://www.cnr.it).

## Overview

The telepresence aspects of the SI-Robotics project aims to develop a social robot that can be used in the context of social inclusion. The robot is able to interact with the user through a natural language interface and to perform actions in the environment in order to help the user in his/her daily activities. The robot is also able to autonomously navigate in the environment and to avoid obstacles.

## Requirements

- The Robot Operating System ([ROS](https://www.ros.org)) is a set of software libraries and tools that help you build robot applications. Follow the [installation instructions](http://wiki.ros.org/ROS/Installation) to install ROS on your system. We recommend to install the latest LTS version of ROS, which is currently [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).

- Duckling is a library that parses text into structured data. It is used to extract entities from the user's utterances. Follow the [installation instructions](https://github.com/facebook/duckling) to install Duckling on your system.

- Spacy is an open-source software library for advanced natural language processing. It is used to extract entities from the user's utterances. Follow the [installation instructions](https://spacy.io/usage) to install Spacy on your system.

- Rasa is an open source machine learning framework to automate text-and voice-based conversations. It is used to understand the user's intent and to generate the robot's responses. Follow the [installation instructions](https://rasa.com/docs/rasa/installation) to install Rasa on your system.

- Android Studio is the official integrated development environment for Google's Android operating system. It is used to develop the Android application that allows the user to interact with the robot. Follow the [installation instructions](https://developer.android.com/studio/install) to install Android Studio on your system.

## Installation

Clone the repository in your ROS workspace and compile it using `catkin_make`:

```bash
git clone --recurse-submodules https://github.com/pstlab/sir_telepresence
catkin_make
```

Compile the Android application, in `android_gui`, using Android Studio and install it on the robot. Once compiled, the application can be installed on the robot using the following command:

```bash
adb install sir_telepresence/android_gui/app/build/outputs/apk/debug/app-debug.apk
```

Train the Rasa model:

```bash
cd sir_telepresence/nlu
rasa train
```

Start the Duckling server:

```bash
stack exec duckling-example-exe
```

Download the Spacy model and import it:

```bash
python3 -m spacy download it_core_news_lg
python3 -c "import spacy; spacy.load('it_core_news_lg')"
```

## Usage

Start the Rasa server:

```bash
cd sir_telepresence/nlu
rasa run --enable-api
```

Start the Rasa actions server:

```bash
cd sir_telepresence/nlu
rasa run actions
```

Start the robot's GUI:

```bash
adb shell am start -n com.pstlab.sir_telepresence/com.pstlab.sir_telepresence.MainActivity
```

Launch the `ohmni.launch` file to start the Ohmni robot:

```bash
roslaunch sir_telepresence ohmni.launch
```