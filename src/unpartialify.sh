#!/bin/bash

cp MiniCopterDrone.cs MiniCopterDrone.cs.copy

node ./unpartialify.js MiniCopterDrone.DroneManager.cs MiniCopterDrone.cs.copy
node ./unpartialify.js MiniCopterDrone.Drone.cs MiniCopterDrone.cs.copy
node ./unpartialify.js MiniCopterDrone.DroneCPU.cs MiniCopterDrone.cs.copy
node ./unpartialify.js MiniCopterDrone.Compiler.cs MiniCopterDrone.cs.copy
node ./unpartialify.js MiniCopterDrone.PIDController.cs MiniCopterDrone.cs.copy

cp MiniCopterDrone.cs.copy ../MiniCopterDrone.cs