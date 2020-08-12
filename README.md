[//]: # (Image References)

[image1]: ./simulation.png "Model Visualization"
[image2]: ./pf_flow.png "Particle filter Flow"

# Overview
This is an implementation of a 2 dimensional particle filter in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

## Running the Code
To use this code with the Udacity Unity-based Simulator, follow the instructions [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. 

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

## Results

The implementation utilized the following flow for the algorithm:

![alt text][image2]

The particle filter used only 50 particles to allow for computational efficiency. 

Using the Udacity simulator, the particle filter maintained localization to within ~0.1m in x and y coordinates. The figure below shows the vehicle; the region of particles; and the predicted and true distances to map landmarks.

![alt text][image1]