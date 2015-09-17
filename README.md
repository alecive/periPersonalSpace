Peripersonal Space
=================

This repository deals with the implementation of peripersonal space representations on the iCub humanoid robot.

## Overall Idea

The basic principle concerns the use of the tactile system in order to build a representation of space immediately surrounding the body - i.e. peripersonal space. In particular, the iCub skin acts as a reinforcement for the visual system, with the goal of enhancing the perception of the surrounding world. By exploiting a temporal and spatial congruence between a purely visual event (e.g. an object approaching the robot's body) and a purely tactile event (e.g. the same object eventually touching a skin part), a representation is learned that allows the robot to autonomously establish a margin of safety around its body through interaction with the environment - extending its cutaneous tactile space into the space surrounding it.

We consider a scenario where external objects are approaching individual skin parts. A volume is chosen to demarcate a theoretical visual "receptive field" around every taxel. Learning then proceeds in a distributed, event-driven manner - every taxel stores and continuously updates a record of the count of positive (resulting in contact) and negative examples it has encountered.

## Video

A video of the capabilities of the software is available here: 

## Repository Structure

### PeriPersonal Space Library

The library is located under the `lib` directory. 

### Modules

## Authors

 * Alessandro Roncone (@alecive)
 * Matej Hoffman (@matejhof)
 * Ugo Pattacini (@pattacini)
