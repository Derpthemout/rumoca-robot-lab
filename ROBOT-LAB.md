# Rumoca Robot Lab

An articulated robotics prototype in SLAM Web at `/robot.html`. The original UAV entry point remains at `/`.

## Run locally

```sh
npm ci
node scripts/assets.mjs
npm run dev:robot
```

On Windows PowerShell use `npm.cmd` if script execution policy blocks `npm`. Robot Lab includes its model and needs no Modelica library submodule. The original UAV examples require `npm run models:init`.

## Try the motion workflow

1. Leave **Move through simulated servos** checked. Expand **Joint targets** and move a slider, or select an end effector, edit its XYZ target in metres, and click **Solve IK target**.
2. Click **Run** to follow the target, **Pause** to edit it, or **Step 20 ms** for a single simulation step. Joint readouts show measured angle → commanded angle. The IK residual describes the solved target; the actual robot follows over time.
3. **Reset episode** restores the episode's initial pose, zero velocity and time zero. **Neutral pose** starts again at the bounded zero pose. Applying servo parameters starts a new episode at the current pose.
4. Attach a depth or LiDAR sensor to a link/site. **Capture sensor** samples once; enable **Capture at 5 Hz** before running for timestamped captures. Images retain their capture timestamp between samples.
5. Under **Reaching benchmark & recordings**, leave seed 42 and left_foot selected, then click **Run seeded reach**. This starts at neutral and runs a deterministic IK target-hold baseline. **Export rollout** saves observations, normalized actions, rewards and termination/truncation flags.
6. **Export motion recording** saves manually run steps and live sensor samples. Recording stops at 3,000 steps (60 simulated seconds); export and reset to continue.
7. **Download Modelica** exports the actual generated equations. **Export experiment** saves profile, commands, servo settings and sensor mount. Import restores a bounded pose at time zero with zero velocity; it does not resume a transient or replay sensor images. `measuredQ` preserves any compliant-stop overshoot separately.

Uncheck simulated servos to edit poses instantly using Rumoca forward kinematics.

## What Rumoca does

Rumoca 0.10.3 compiles the generated Modelica and evaluates every forward-kinematics query. In motion mode it also integrates the second-order joint servo equations, torque saturation and compliant joint stops. JavaScript supplies targets and computes bounded position IK using finite differences of Rumoca evaluations; it does not integrate substitute robot dynamics. Compilation and simulation run in a worker.

The generic versioned JSON profile supports revolute trees, offset pivots, frame rotations, sites and visuals. Included Microduck geometry has 14 modeled actuated hinges and published STL meshes. This is not a general URDF/MJCF importer.

`src/robotics/reach-environment.ts` exposes `ReachEnvironment.reset(options)` and `step(action)`. Each action is one absolute target per joint normalized to [-1,1]. Observations include angles, velocities, goal, end-effector position and simulation time. Reward is distance progress minus time-weighted distance and squared actuator torque, plus a success bonus. Success requires <1 mm position error and joint speeds <0.05 rad/s; timeout is reported separately. The baseline uses IK, not a learned policy.

## Fidelity boundaries

In the joint lab, the root is fixed. Each joint has independent, approximate inertia and servo parameters; these are teaching defaults, not identified Microduck motors or coupled rigid-body dynamics. Stops are compliant and may briefly overshoot. Excessive penetration faults the simulation until reset. The joint lab has no gravity or ground contact. The separate reduced-order stepping experiment below adds translational body/contact dynamics, but not unconstrained balance, coupled multibody dynamics, collision-aware IK or orientation IK.

Ideal depth/LiDAR rays intersect a demonstration scene. Ranges are radial metres, with +X forward / +Y left / +Z up. There is no noise, latency, robot self-occlusion or calibrated hardware sensor model. Convert radial to axial depth before using an axial-depth SLAM contract.

The reaching reset/step interface is a foundation for training experiments. No policy optimizer, learned policy, SLAM connection, Spot model or real-robot control is included. The existing UAV estimator is not connected to this page.

## Validation

`npm run test:robot` runs 26 tests against real Rumoca WASM, covering FK/IK, offset pivots, sensor transforms/ranges, dynamic response, torque caps, reset/replay and reaching episodes. `npx tsc --noEmit` checks the full project. A full production build uses `npm run build` after installing assets and initializing the Modelica submodule.

See `docs/evidence/robot-lab-validation.md` for recorded checks and their limits.

## Sources

- SLAM Web: https://github.com/CogniPilot/slam_web
- Rumoca: https://github.com/CogniPilot/rumoca (pinned 0.10.3)
- Microduck model/meshes: https://github.com/pollen-robotics/microduck_rl
- Microduck runtime: https://github.com/pollen-robotics/microduck

Licenses and content hashes are in `public/robots/`. Upstream XML is preserved unchanged; its inertial and collision semantics are not imported into the simplified servo model.

## Mapping and route planning

1. Open **Mapping & route planning** and click **Set level mapping LiDAR**. This mounts a horizontal LiDAR 0.1 m above the root and clears the previous map.
2. Click **Capture sensor**. The map shows observed free cells in light teal, occupied cells in dark ink and unknown cells in gray. The small cyan marker is world origin. Live 5 Hz captures also accumulate when enabled.
3. Keep start `(0, 0)`, goal `(-0.5, 0)` and footprint radius `0.1 m`, then click **Plan route**. An orange route appears through observed free space. Change the coordinates to explore other goals. Unknown space and insufficient footprint clearance cause a clear failure instead of a speculative route.
4. **Export map** saves the 5 cm, 6 × 6 m grid, bounded evidence scores, scan height, pose-source annotation and planned cell-center route with footprint radius. **Clear map** begins a new mapping session. Importing a robot clears its map.

The ground-truth mode uses sensor transforms from Rumoca; occupancy accumulation and grid search run in TypeScript. The estimated-pose mode below adds local LiDAR odometry, but neither mode closes loops. Depth images are not fused. Tilted LiDAR and scan-height changes are rejected to avoid mixing different horizontal slices. Use **Clear map** before deliberately changing scan height.

The planner uses equal-cost four-neighbor search with circular-footprint inflation and blocked unknown cells. Its result is a grid-center path, not a trajectory, gait or motor command. The scan-height slice can miss low obstacles, overhangs and unsupported ground. There is no full-body collision check or execution of the route. A single stationary scan has limited coverage; moving a sensor offset adds known-pose observations, not robot locomotion.


## LiDAR odometry and Rumoca sensor sweep

Click **Run Rumoca sensor sweep** under Mapping & route planning. Rumoca integrates a separate virtual sensor rig for 2 seconds (41 samples), moving it 6 cm in X, 2 cm in Y and turning 0.04 radians. The robot remains fixed. Range scans feed a local scan matcher, which estimates motion and builds the occupancy map. Cyan is ground truth; pink is the estimate. Drift, fit residual and matched-return overlap appear below the button. **Plan route** works on the resulting estimated-pose map.

**Export map** includes estimated/truth pose histories, timestamps, drift, residuals, overlap and the sweep's generated Modelica. Clear the map before returning to mounted sensor captures; mixing the two streams is blocked.

For manual experiments, select **LiDAR odometry (first pose anchored)**, set a level mapping LiDAR and capture. Make small sensor-offset or yaw changes and capture again. The first valid scan uses the known Rumoca pose as the map anchor. Later optimizer updates receive only radial ranges, never ground-truth XY/yaw or world hit points. Truth remains available separately for diagnostics. Height and horizontal orientation are validated using the simulation's sensor frame.

The matcher uses iterative closest surface-segment alignment with trimmed correspondences, bounded movement, minimum overlap and residual checks. Tracking failure stops fusion until **Clear map**; it does not silently substitute ground truth. Feature-poor, repetitive or symmetric geometry and large inter-scan motion can fail or yield plausible but incorrect estimates. These heuristic quality checks do not guarantee localization accuracy.

This is local LiDAR odometry plus occupancy mapping, not a complete SLAM backend. There is no loop closure, global relocalization, pose graph, uncertainty propagation, learned policy or walking controller. Rumoca owns the sensor sweep and robot physics; TypeScript implements this scan matcher. The original UAV SLAM pipeline is not reused by this prototype.


## Ground contact and stepping

Click **Run four steps** in **Ground contact & stepping**. Unlike the sensor sweep, this moves the rendered robot: it begins from a crouched, narrowed stance, settles onto the ground, transfers support, and alternates four short steps. **Stop** freezes after the current step. **Return to joint lab** restores the saved normal robot state. **Export stepping** saves the contact/servo Modelica, per-step body and joint states, foot positions and controller phase.

Rumoca integrates gravity, compliant vertical foot forces and a friction-limited horizontal centroidal controller. The joint servos and contact body advance as separate models at 20 ms intervals. Assumed parameters are 0.6 kg mass, 1500 N/m contact stiffness, 20 Ns/m contact damping and friction coefficient 0.6. They are illustrative, not calibrated Microduck parameters. Each displayed support patch is an approximate axis-aligned 6 × 3.6 cm rectangle; the margin readout uses the bounding box of active patches, not a full support-polygon balance proof.

The controller waits for support before lifting a foot, checks touchdown before changing support, and stops on unreachable targets, support timeout or excessive falling speed. The demonstration requests 15 mm advance and 12 mm lift per step. The tested run completed in 10.5 simulated seconds, lifted both feet about 11.7 mm, advanced each foot about 30 mm over two steps, and held commanded foot IK residual below 0.5 mm.

**This is a reduced-order stepping experiment, not validated free balancing.** Body orientation is constrained upright. Contact forces do not feed back into the independent joint torque dynamics, foot orientation and full sole collisions are not enforced, and articulated inertia, angular momentum, ground slip and tipping are not modeled. There is no route following, terrain traversal or hardware command output. The separate mapping and odometry workflows remain available after returning to the joint lab.
