---
theme: ./
layout: cover
hideInToc: true
class: text-white
coverAuthor: 肖飞宇
coverBackgroundUrl: /science.jpg
fonts:
  local: Montserrat, Roboto Mono, Roboto Slab # local fonts are used for legal reasons for deployment to https://slidev-theme-academic.alexeble.de and only set up for the example project, remove this line for your project to automatically have fonts imported from Google
themeConfig:
  paginationX: r
  paginationY: t
  paginationPagesDisabled: [1]


---

# 量产自动驾驶中的决策规划

## 从领航高速(NOP)到领航城区(City-NOP)


---
layout: intro
hideInToc: true
---

# 引言

## 自动驾驶中的决策规划



---
layout: default
---

# Planning and Decision Making

<img src="/pipeline.png" width = "700">

## Problem Definition
+ Goal: find and follow path from current location to destination
+ Take static infrastructure and dynamic objects into account
+ Input: vehicle and environment state (via perception stack)
+ Output: Trajectory as input to vehicle controller

## Challenges:
+ Driving situations and behaviors are very complex
+ Difficult to model as a single optimization problem

---
layout: default
hideInToc: yes
---

# Planning and Decision Making

<img src="/review.png" width = "800">


+ Idea: Break planning problem into a **hierarchy of simpler problems**

+ Each problem tailored to its scope and level of abstraction

+ Earlier in this hierarchy means higher level of abstraction

+ Each optimization problem will have constraints and objective functions

<Footnotes separator>
  <Footnote :number=1>Paden et al.: A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles. TIV, 2016.</Footnote>
</Footnotes>

---
layout: default
hideInToc: yes
---

# Planning and Decision Making

<img src="/review.png" width = "800">


**Hierarchy**: A destination is passed to a route planner that generates a route through the road network. A behavioral layer reasons about the environment and generates a motion specification to progress along the selected route. A motion planner then solves for a feasible motion accomplishing the specification. A feedback control adjusts actuation variables to correct errors in executing the reference path.

---
layout: default
hideInToc: yes
---

## Step 1: Route Planning

<img src="/review.png" width = "800">


+ Represent **road network** as **directed graph**.

+ Edge weights correspond to road segment length or travel time

+ Problem translates into a minimum-cost graph network problem

+  Inference algorithms: Dijkstra, $A^{*}$

**Solved** for industrial applications like Map Apps.

<Footnotes separator>
  <Footnote :number=1>Delling, Daniel et al. “Customizable Route Planning.” The Sea (2011).</Footnote>
</Footnotes>

---
layout: default
hideInToc: yes
---

## Step 2: Behavior Planning

<img src="/review.png" width = "800">


+ Select **driving behavior** based on current vehicle/environment state

+ E.g. at stop line: stop, observe other traffic participants, traverse

+ Often modeled via **finite state machines** (transitions governed by perception)

+ Can be modeled probabilistically, e.g., using Markov Decision Processes (MDPs)

---
layout: default
hideInToc: yes
---

## Step 3: Motion Planning

<img src="/review.png" width = "800">


+ Find feasible, comfortable, safe and fast vehicle trajectory

+ Exact solutions in most cases computationally intractable

+ Often numerical approximations are used

+ Approaches: variational methods, graph search, incremental tree-based

<Footnotes separator>
  <Footnote :number=1> Note: find feasible not optimal trajectory. </Footnote>
</Footnotes>

---
layout: default
hideInToc: yes
---

## Step 4: Local Feedback Control

<img src="/review.png" width = "800">


+ **Feedback controller** executes the trajectory from the motion planner

+ Corrects errors due to inaccuracies of the vehicle model

+ Emphasis on **robustness, stability and comfort**

+ *Open-loop planning* plus *feedback control* gets robust excuation.

<Footnotes separator>
  <Footnote :number=1> Note: find feasible not optimal trajectory. </Footnote>
</Footnotes>

---
layout: intro
---

# Behavior Planning
## An overall introduction


---
layout: default
hideInToc: yes
---

## Behavior Planning

+ To follow a planned route, the vehicle must conduct **various maneuvers**

+ Examples include: speed tracking, car following, stopping, merging, etc.

+ It is difficult to design a motion planner for all maneuvers jointly

+ The **behavior planning** stage thus **discretizes the behaviors** into simpler (atomic) maneuvers, each of which can be addressed with a dedicated motion planner

+ The behavior layer must take into account traffic rules, static and dynamic objects

+ **Input**: High-level route plan and output of perception stack

+ **Output**: Motion planner constraints: corridor, objects, speed limits, target, ...

+ Frequently used models:
  +  Deterministic: Finite State Machines (FSMs) and variants
  +  Probabilistic: Markov Decision Processes(RL related)

---
layout: default
hideInToc: yes
---

## Finite State Machine

A Finite State Machine (FSM) is defined by quintuple $(\sum, \mathrm{S}, \mathrm{F}, s_0, \delta)$

+ $\sum$ is the input alphabet

+ $\mathrm{S}$ is a non-empty set of states

+ $\mathrm{F} \subset \mathrm{S}$ is the set of final states

+ $s_0 \in \mathrm{S}$ is the initial state

+ $\sigma: \mathrm{S} \times \mathrm{\sum} \to \mathrm{S}$ is the state transition function

Example: 
<img src="/fsm.jpg" width = "500">

---
layout: default
hideInToc: yes
---

## FSM for a Simple Vehicle Behavior

<img src="/fsmexample.png" width = "700">

---
layout: default
hideInToc: yes
---

## Handling Multiple Scenarios

<img src="/fsmmul.png" width = "700">

+ Hierarchical State Machine (HSM)

+ Advantages: Simpler, more efficient – Disadvantages: Rule duplication



---
layout: default
hideInToc: yes
---

## Example from DARPA Challenge

<img src="/darpa.png" width = "800">

<Footnotes separator>
  <Footnote :number=1> Gindele et al.: Design of the planner of team AnnieWAY’s autonomous vehicle used in the DARPA Urban Challenge 2007. IV, 2008. </Footnote>
</Footnotes>

---
layout: default
hideInToc: yes
---

## Summary Finate State Machines



+ Elegant way to break complex behaviors into simpler maneuvers

+ Interpretable and easy to design

+ Rule explosion when dealing with complex scenarios

+ **Cannot handle noise / uncertainty** $\to$ MDPs

+ **Expert-designed hyperparameters** $\to$ Reforcement Learning

Widely used in autonomous systems for highway driving(in most cases), not suitable for urban driving.

Note: FSM based decision system is usually denoted as rule-based decision system.


---
layout: intro
---

# Motion Planning
## An overall introduction



---
layout: default
hideInToc: yes
---

## Motion Planning

**Goal**:
+ Compute safe, comfortable and feasible trajectory from the vehicle’s current configuration to the goal based on the output of the behavioral layer
+ Local goal: center of lane a few meters ahead, stop line, parking spot
+ Takes as input static and dynamic obstacles around vehicle and generates collision-free trajectory

**Focus on Trajectory not only Path**
+ Path: $\sigma(l): [0,1] \to \mathcal{X}$ (does not specify velocity)
+ Trajectory: $\pi(t): [0,T] \to \mathcal{X}$ (explicitly considers time)
+ **Completeness of planning**: in the configuration space $\mathcal{X}$ of the vehicle and $T$ the planning horizon

---
layout: default
hideInToc: yes
---

## Motion Planning

**Main Formulations**:

+ Variational Methods

+ Graph Search Methods

+ Incremental Search Techniques


---
layout: default
hideInToc: yes
---

## Variational Methods

Variational methods minimize a functional:

$$
\begin{aligned}
\underset{\pi}{\operatorname{argmin}} & J(\pi)=\int_{0}^{T} f(\pi) d t \\
\text { s.t. } & \pi(0)=\mathbf{x}_{\text {init }} \wedge \pi(T) \in \mathbf{X}_{\text {goal }}
\end{aligned}
$$

+ The functional integrates soft constraints (spatial, velocity, jerk, etc.)

+ Additional hard constraints can be formulated (minimum turn radius, etc.)

+ Solved using numerical optimization

+ Often nonconvex problem $\to$  **converges slowly to local optimal or even not converge**

Note: that's the reason why we need good hierarchy, namely, if we have better decision inputs to formulate the state space into sub-convex space, the problem can be numerically solved.




---
layout: two-cols
---

# Left
Convex cases can reach global minimum.
<img src="/convex.png" width = "350">

::right::

# Right

Nonconvex cases only fall into local minimum.
<img src="/nonconvex.png" width = "350">

---
layout: default
hideInToc: yes
---

## Graph Search Methods

<img src="/lattice.png" width = "900">

+ Idea: Discretize configuration space into graph

+ Various algorithms for constructing graphs

+ Search strategies:  Dijkstra, $A^{*}$, ... (like *route planning*)


---
layout: default
hideInToc: yes
---

## Graph Search Methods： Examples


<html>
<body>
<table><tr>
<td><img src="/Astar.gif" width="500"   border=0 /></td>
<td><img src="/Dijkstra.gif" width="500"   border=0 /></td>
</tr></table> 
</body>
</html>

---
layout: default
hideInToc: yes
---

## Graph Search Methods： Examples

<html>
<body>
<table><tr>
<td><img src="/D_star.gif" width="500"   border=0 /></td>
<td><img src="/LPAstar.gif" width="500"   border=0 /></td>
</tr></table> 
</body>
</html>

---
layout: default
hideInToc: yes
---

## Incremental Search Techniques



+ Idea: Incrementally build increasingly finer discretization of configuration space

+ Guaranteed to provide feasible path given enough computation time

+ But: computation time can be unbounded

+ Prominent example: Rapidly exploring random trees (RRTs)

Note: Not complete and usually suitable for vehicle motion planning.

<html>
<body>
<table><tr>
<td><img src="/RRT_2D.gif" width="250"   border=0 /></td>
<td><img src="/RRT_STAR2_2D.gif" width="250"   border=0 /></td>
</tr></table> 
</body>
</html>


---
layout: figure-side
figureCaption: Practical Search Techniques in Path Planning for Autonomous Driving. STAIR, 2008.
figureFootnoteNumber: 1
figureUrl: hybrida.png
---

## Hybrid $A^{*}$


+ Hybrid A* is an A* variant that guarantees kinematic feasibility of the path

+ Planning is re-applied continuously as the car explores the environment

+ A practical method for **kinematic motion planning**

**Kinodynamic : Kinematic + Dynamic**

+ Coarse-to-fine proces

+ Trajectory only optimizes locally

+ Infeasible path means nothing to nonholonomic system

---
layout: default
---
# Summary

+ Driving situations and behaviors are very complex

+ Thus, we break the problem into a hierarchy of simpler problems:
  +  Route planning, behavior planning and motion planning
  +  Each problem is tailored to its scope and level of abstraction

+ A* exploits planning heuristics to improve efficiency
+ Behavior planning can be implemented using finite state machines
+ For motion planning, variational and graph search methods are often used


---
layout: intro
hideInToc: true
---

# 量产 L2+ 中的决策规划算法实践
## 以高速领航辅助驾驶(NOP)为例


---
layout: intro
hideInToc: true
---

# 量产自动驾驶中的决策规划挑战
## 基于城区领航辅助驾驶(City-NOP)分析