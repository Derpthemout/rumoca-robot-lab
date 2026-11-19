# Robot Lab validation — 2026-11-18

Implemented in a local checkout of CogniPilot/slam_web. No upstream branch, PR, or GitHub project was created.

- Full-project TypeScript check passed.
- Production Vite build passed with both index.html and robot.html. Existing large-bundle warnings remain.
- Real Rumoca WASM tests passed for Microduck FK/IK and offset-pivot transforms. Reachable IK acceptance: <=0.5 mm; unreachable goals remain finite and are marked unsuccessful.
- In-app browser: loaded Microduck through Rumoca 0.10.3; changed left-foot X target from -0.03178 m to -0.026 m; observed target reached with 0.19 mm residual in one iteration.
- In-app browser: default depth capture produced 559 returns; switching to 128-ray LiDAR produced 35 returns. These are observations for the demonstration scene and tested pose, not sensor accuracy benchmarks.
- Headless Edge test could not start because Windows closed its sandboxed process. Browser checks used the app's browser instead.
- No dynamic contact, gait, training, SLAM, Spot hardware, or sim-to-real validation has been performed.

The checked compiler remains the repository's pinned Rumoca 0.10.3 package. No compiler API or derivative support was assumed beyond methods exercised by the tests.

## Motion milestone — 2026-11-19

- Added Rumoca-integrated independent joint servos, saturation and compliant stops; measured and commanded joint states are separate.
- All 11 tests passed across tests/robotics.test.ts and tests/robot-motion.test.ts using the real pinned Rumoca WASM compiler. Includes gradual response, reset/replay, torque-cap effects, invalid-input isolation, moving Microduck camera frames, deterministic reaching, timeout/success distinction and seed-42 Microduck left-foot success below 1 mm.
- Full-project TypeScript check and production Vite build passed. Existing large-bundle warnings remain.
- Added Run/Pause/20 ms step, timestamped 5 Hz ideal sensor sampling, bounded motion recordings, seeded reaching rollouts and settings export/import.
- A fresh automated browser smoke check could not launch Edge: Windows closed the sandboxed browser with a crashpad named-pipe error. This milestone's UI has not been interactively revalidated; prior browser observations above apply to the kinematic milestone only.
- The reaching controller is an IK target-hold baseline, not a learned policy. Servo parameters are approximations, not calibrated hardware. No gravity/contact, balance, locomotion, SLAM or sim-to-real validation is claimed.

## Mapping milestone — 2026-11-19

- Added a known-pose horizontal LiDAR occupancy grid (6 m square, 5 cm cells), map canvas, export and conservative four-neighbor route planning with circular footprint inflation. Rumoca supplies the sensor frame; TypeScript implements occupancy accumulation and search.
- All 17 robot tests passed across the three test files in the final combined run. Full-project TypeScript and the production Vite build also passed; existing large-bundle warnings remain.
- New checks cover occupied endpoints/free interiors/unknown regions, malformed and tilted scans without mutation, yaw transforms, deterministic mapping, scan-height consistency, obstacles, unknown-space refusal and insufficient footprint clearance.
- End-to-end fixture evaluates the real Microduck root frame in Rumoca, mounts a LiDAR, raycasts demonstration obstacles, integrates the scan and successfully plans the default `(0,0)` to `(-0.5,0)` route with 0.1 m radius.
- No SLAM estimation, full-body collision checking or walking execution is claimed. This milestone has not received an interactive browser check.


## Local odometry milestone — 2026-11-19

- Added range-only local LiDAR odometry using trimmed closest surface-segment alignment, overlap/residual/motion bounds and geometry checks. Only the initial map anchor uses known XY/yaw; truth remains separate for diagnostics and planar-scan validation.
- Added estimated-pose map fusion, latched tracking-loss behavior, truth/estimated trajectory overlays and drift/residual/overlap readouts. Exports include pose histories and timestamps. No loop closure or global relocalization is claimed.
- Added a Rumoca-integrated virtual sensor sweep: 2 seconds, 41 scans, 6 cm X / 2 cm Y / 0.04 rad yaw. This moves a virtual rig, not the robot. Room walls provide sufficient geometry in the demo.
- Tests verify small translation/yaw recovery, bad-input recovery, arbitrary initial anchors, invariance to later ground-truth pose changes, and frozen maps after tracking failure. The real Rumoca-to-raycast-to-odometry integration also checks a route through the estimated map.
- Headless Edge browser validation passed against the local Vite server: sweep reached 41 scans, no page errors, route preview contained 11 cell centers, and downloaded JSON contained 41 pose records plus the generated Modelica. Observed UI drift: position 0.0 mm at one-decimal display precision, yaw 0.01 degrees, residual 0.3 mm, overlap 100%. These are ideal synthetic-scene results, not real sensor accuracy.
- Initial sandboxed browser/localhost checks were blocked by the environment. Running the local check outside the sandbox succeeded after starting the dev server.
- Final verification: all 23 tests passed, full-project TypeScript check passed, and the production build passed with existing large-bundle warnings. The browser sweep/route/export check also passed after the timestamp changes.

## Reduced-order stepping milestone — 2026-11-19

- Added Rumoca gravity/contact dynamics, nonnegative compliant normal forces and friction-limited horizontal force. The body is constrained upright; this is not coupled articulated dynamics or validated balance.
- Four-step controller uses a reachable crouched/narrowed initial stance, support-transfer and touchdown gates, position IK, and bounded independent joint servos. It stops on unreachable targets, support timeouts or excessive falling speed.
- New tests cover gravitational fall, settled normal force equal to weight, friction bound, zero contact/tangential force in flight, invalid-input isolation, and four alternating foot lifts/advances.
- Stepping test observed 4 steps / 10.5 simulated seconds, both feet lifted about 11.7 mm, each advanced about 30 mm, maximum commanded IK residual below 0.5 mm, and final total contact force about 5.885 N for the assumed 0.6 kg body.
- Headless Edge passed Run, Stop, Return to joint lab, full four-step completion, export, and return after completion, without page errors. Export contained 525 contact states and matching joint records plus both generated Modelica sources (~1.33 MB). Screenshot saved at outputs/rumoca-stepping-preview.png outside the checkout.
- This does not validate tipping, sole orientation/contact meshes, articulated momentum, slipping, terrain, route execution or real hardware.
