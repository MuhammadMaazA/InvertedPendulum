```markdown
# SysML Models for Inverted Pendulum System

This directory contains the Model-Based Systems Engineering (MBSE) artifacts for the inverted pendulum system, created using Eclipse Papyrus.

## Overview

We used the Systems Modeling Language (SysML) to define the system architecture, behavior, and requirements. This approach helped ensure a systematic design process with clear traceability between requirements and implementation.

## File Structure

- `InvertedPendulumModel.di`: Eclipse Papyrus diagram information file
- `InvertedPendulumModel.notation`: Graphical notation file
- `InvertedPendulumModel.uml`: UML model file containing the actual model data

## SysML Diagrams

### Structure Diagrams
- **Block Definition Diagram (BDD)**: Defines the system components including Cart, Pendulum, Actuator, Sensor, Controller, and Disturbance blocks
- **Internal Block Diagram (IBD)**: Shows the connections and interfaces between system components
- **Package Diagram**: Organizes the model elements into logical packages

### Behavior Diagrams
- **Activity Diagram**: Depicts the control flow from system startup through stabilization and disturbance response
- **State Machine Diagram**: Shows the system states (Idle, Monitoring, Computing Control, Applying Force, Stable, Unstable)
- **Sequence Diagram**: Illustrates the interactions between system components during operation

### Requirements Diagrams
- **Requirements Diagram**: Captures the functional and non-functional requirements
- **Parametric Diagram**: Defines the mathematical constraints and relationships

## Key Model Elements

- **System Properties**:
  - Pendulum length: 0.3m
  - Pendulum mass: 0.8kg
  - Cart mass: 0.5kg
  - Gravity: 9.81 m/s²

- **Controller Parameters**:
  - PID controller with Kp, Ki, Kd gains
  - Response time requirement: ≤ 0.3s

- **Sensor Specifications**:
  - Motion sensor accuracy: ±0.01m
  - Pendulum angle sensor accuracy: ±0.35°
  - Sampling rates: 200Hz for motion, 50Hz for angle

## Viewing the Models

To view these models, you need Eclipse Papyrus. Follow these steps:
1. Install Eclipse Papyrus (https://www.eclipse.org/papyrus/download.html)
2. Open Eclipse with the Papyrus perspective
3. Import the project into your workspace
4. Double-click on any .di file to open the associated diagrams