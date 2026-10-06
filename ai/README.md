# Wildhat AI

## Overview

This is the Team 100 Wildhat AI system. It consists of tools for training and
evaluating robotics AI. The goal of this project is to create AI for performing
tasks needed by FRC robots: vision, perception, planning and control.

- **Simulator:** physics engine, rendering and data logging
- **Training Data Generation:** auto-labeled data with ground truth
- **Training:** model training, both supervised and RL
- **Evaluations:** closed- and open-loop model testing
- **Visualizer:** a browser-based app for viewing model inputs and outputs
- **Deployment:** deploying and running models on a real robot

## Simulator

The simulator includes a physics engine, a static environment, static objects,
dynamic objects and a skybox. All objects in the scene support domain
randomization to help bridge the sim-to-real gap.

The `Scene` class describes the static environment. This includes the geometry
of the scene, static objects and the skybox.

The `Scenario` class adds more static objects, dynamic objects and actors to the
scene. The `Scenario` also specifies parameters like duration and a random seed
which is saved for reproducibility. The same `Scene` and `Scenario` are used to
generate all episodes in a simulation run, but the random seed is different for
each episode.

`Actors` are dynamic objects in the scene that include control policies. Examples
include robots, people and vehicles. The control policy is configurable and can
be implemented by either a learned model or scripted behavior. The hardware and
sensors attached to each actor can be specified in the actor code.

The `Logger` class controls what data is logged while the episode is running.
The simulator supports the export of sensor data, ground-truth state of all
objects in the scene and other ground-truth annotations like depth maps and
occupancy grids. The `Logger` can log data from sensors attached to the scene or
to any actors in the simulation. Logged data can be written to disk and also
streamed in real time over a network connection. Which data streams are
logged and streamed can be specified in the `Logger`'s configuration.

The `PostProcessor` class controls what steps are taken after the episode
completes, to post-process the data logged during a simulation. For example,
exported images can be compiled into videos.

Every run of the simulator includes one or more episodes. These runs and
episodes are recorded in the `runs` database, along with the data exported from
each.

## Training Data Generation

Simulations can be run specifically to generate auto-labeled training data.
These runs are recorded in the `runs` database and the logged data is stored for
future training runs.

## Training

Supervised training jobs require a dataset built from the `runs` database.
The dataset specifies episodes to include in the training and eval sets,
allowing this split to be consistent across training runs.

Reinforcement learning jobs support multi-actor rollouts that feed multiple
learner models in parallel. Each actor in a rollout is controlled by one policy,
and different actors may run different policies — so several models, alongside
scripted behaviors, can participate in a single rollout. The reward function
lives in the training code, which also decides which actors' trajectories to
include when updating weights between rollouts.

Training jobs export metrics that can be monitored as training progresses, for
example eval set losses or reward function scores.

Training runs, along with their dataset, metrics and checkpoints, are tracked in
the `training` database.

## Evaluations

There are two types of evaluations: open-loop and closed-loop.

**Open-loop evals** do not run a simulation, but instead use the output from a
previous simulation as input to a model. These evals are recorded in the `runs`
database and the run that originally generated the input data is referenced as
the parent.

**Closed-loop evals** launch a new simulation run with a model included as one
or more actors' control policies. These evals are also logged in the `runs`
database. Metrics like success rate are saved for each closed-loop eval.

The `runs` database includes references to the `training` database for each
model used in an evaluation.

## Visualizer

The visualizer app displays a filterable list of `runs` and allows the data
exported from each episode to be viewed. The app can also receive live streams
from the `Logger` running in simulation or on a real robot and display them
the same way it displays data from previous `runs`. The layout of the
visualizer's widgets can be modified dynamically by the user. The basic widget
types are video playback and time series signal plots. Each plot can be
configured to display one or more signals. The entire screen can be split to
compare two episodes side-by-side.

Playback controls allow the user to play/pause, scrub and change playback speed.
A playhead is depicted on the signal plots, in sync with video playback. All
videos are synchronized.

## Deployment

Model export tools prepare models in the `training` database for common robotics
deployment targets like Raspberry Pi or Jetson.

The inference app loads an exported model and executes it on the robot. It can
be configured with a `Logger` to log and stream model inputs and outputs.
Logged data can be uploaded from the robot. The upload tool can be configured
with a `PostProcessor` to run before the data is uploaded. The upload records
the data to the `runs` database, so it can be viewed in the visualizer and
used for future open-loop evals.
