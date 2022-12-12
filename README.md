# LEGO MINDSTORMS EV3 Self-Driving Car Project 

Drive system software for autonomous vehicles built with LEGO MINDSTORMS EV3

Implementation project for Embedded Software Design(ITE4067) Class (Hanyang University, Fall Semester 2022)

## Features

The drive system implements several types of strategies for autonomous driving scenario.

### Driving

The vehicle scans both sides of the lane and moves along the lane.

### Road Surface Marking Detection

While driving, the vehicle detects road markings.

The vehicle supports several types of road markings.

* Pause sign(Blue Block) - Stop driving and wait 3 seconds. 
* School zone sign(Yellow Block) - Drive at half speed until the end of the school zone.
* End of the lab sign(Red Block) - Start examining the parking lot indicator to park.

### Obstacle Detection on The Road

If an obstacle is detected on the road while driving, the vehicle changes to the left or right lane.

### Parking Zone Detection

After completing one lap, parking is performed when a parking lot indicator is found.

## Platooning Modes

The drive system support three platooning modes: PLATOONING_OFF, PLATOONING_LEADER, PLATOONING_FOLLOWER

### PLATOONING_OFF

In PLATOONING_OFF mode, The vehicle drives alone without platooning.

The vehicle detects all objects around the road on its own and takes action accordingly.

### PLATOONING_LEADER

In PLATOONING_LEADER mode, the vehicle acts as a leader during platooning.

During platooning, all vehicles exchange signals through a Bluetooth network.

The driving strategy and maneuvering strategy against obstacles are the same as those of PLATOONING_OFF mode. Additionally, in PLATOONING_LEADER mode, a signaling system has been added to allow simultaneous response to obstacles from following vehicles.

### PLATOONING_FOLLOWER

In PLATOONING_FOLLOWER mode, the vehicle acts as a follower during platooning.

The following vehicle uses data from the distance sensor to adjust its speed to maintain a consistent distance from the vehicle in front. In addition, instead of detecting an obstacle with a sensor, it receives a signal from the preceding vehicle and responds to the obstacle at the same time.