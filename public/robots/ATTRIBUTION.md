# Microduck model attribution

The included `microduck-walk.xml` is from Pollen Robotics' microduck_rl project:
https://github.com/pollen-robotics/microduck_rl/blob/main/src/mjlab_microduck/robot/microduck/robot_walk.xml

Retrieved 2026-11-18. Its SHA-256 is recorded in microduck.json. Original Apache-2.0 license is included as MICRODUCK-LICENSE. microduck.json is a derived kinematic profile containing the body hierarchy, hinge axes/ranges, and site transforms. Visual STL meshes from the same source directory are included for display, with individual SHA-256 hashes in mesh-hashes.json. Actuator dynamics, mass/inertia, contacts, and the floating root are not simulated by this initial fixed-base kinematics lab. The source lists 14 actuated hinges; do not equate that count with the product's total motor count.
