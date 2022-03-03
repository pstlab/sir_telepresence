## Pull the images from Docker Hub
The dialogue manager requires two docker containers, created from two different images. First, pull the images from Docker Hub

```
docker pull rasa/duckling
docker pull pstlab/dialogue_manager
```

## Creating a Docker network
Each Docker container has its own ip address assigned by Docker. Creating a network allows to refer to the container's name in place of its ip. First step is to create a network.

```
docker network create nlp-net
```

## Creating and running the containers
Since it is referred from the dialogue manager container, the Duckling container must have the `duckling` name. The container, furthermore, must use the common network stack.

```
docker run --net nlp-net --name duckling rasa/duckling
```

Similarly, the dialogue manager container must use the common network stack. The 5005 port must be accessible from the host.

```
docker run --net nlp-net --name dialogue_manager -p 5005:5005 pstlab/dialogue_manager
```

## Creating a new image
An image can be build starting from a Dockerfile. The image must have the same name of the Docker Hub repository (i.e., `pstlab/dialogue_manager`).

```
docker build -t pstlab/dialogue_manager .
```

## Pushing the image on Docker Hub
The image can then be pushed on Docker Hub. First login

```
docker login
```

and then push the image

```
docker push pstlab/dialogue_manager
```
