# Lego Mindstorm EV3 Self-Driving Car Project 

Drive system software for autonomous vehicles built with LEGO MINDSTORMS EV3

Implementation project for Embedded Software Design(ITE4067) Class (Hanyang University, Fall Semester 2022)

## Features

The drive system support three modes: PLATOONING_OFF, PLATOONING_LEADER, PLATOONING_FOLLOWER

### PLATOONING_OFF

### PLATOONING_LEADER

In PLATOONING_LEADER mode, the vehicle acts as a leader during platooning.

During platooning, all vehicles exchange signals through a Bluetooth network.

The driving strategy and maneuvering strategy against obstacles are the same as those of PLATOONING_OFF mode. Additionally, in PLATOONING_LEADER mode, a signaling system has been added to allow simultaneous response to obstacles from following vehicles.

### PLATOONING_FOLLOWER

In PLATOONING_FOLLOWER mode, the vehicle acts as a follower during platooning.

The following vehicle uses data from the distance sensor to adjust its speed to maintain a consistent distance from the vehicle in front. In addition, instead of detecting an obstacle with a sensor, it receives a signal from the preceding vehicle and responds to the obstacle at the same time.