# multi-robot-vicsek
The software uses MPI (a standard supercomputing computing library) and [DroneKit](http://dronekit.io/) to allow a set of vehicles (drones, rovers, etc) to move as a "bird's swarm" using a simplified version of the [Vicsek model](https://link.springer.com/article/10.1140/epjb/e2008-00275-9). 

Each vehicle is to be executed on an independent computer, virtual machine, companion computer etc., running [ArduPilot SITL](http://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) or [MAVProxy](http://ardupilot.github.io/MAVProxy/html/index.html) correspondingly. Moreover, all the computers are to be set as a [High Performance Robotic Computing](https://www.sciencedirect.com/science/article/pii/S092188901830232X) Cluster, see [here](https://github.com/leonardocfor/multi-robot-vicsek) for the configuration of a simple HPRC cluster

The software can be used in real vehicles running ArduPilot (e.g. PX4 boards, etc) with little modification. 

## License

To use this software, please cite the article [Towards High Performance Robotic Computing](https://www.sciencedirect.com/science/article/pii/S092188901830232X) 

## Disclaimer

The user of this software accepts that possible damages to moving vehicles can occur and release the author(s) of this software of any complaint or reclamation.

1. *Simplified Vicsek model*: The software does not currently consider the noise feature in the Vicsek model 
2. *Required modifications* for real vehicles: The software does not currently include conflict detection
