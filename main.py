import pybullet as p
import time
import math

# Connect to the PyBullet physics server
client = p.connect(p.GUI)

# Set the time step and the number of iterations per time step
p.setTimeStep(1 / 60)
p.setPhysicsEngineParameter(numSolverIterations=10)

# Set the gravity
p.setGravity(0, 0, -10)

# Create a floor
floor_id = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, floor_id)

# Create a box, a sphere, and a cylinder
box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
sphere_id = p.createCollisionShape(p.GEOM_SPHERE, radius=0.5)
cylinder_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.5, height=1)

# Set the box's, sphere's, and cylinder's initial position and orientation
p.resetBasePositionAndOrientation(box_id, [0, 0, 1], [0, 0, 0, 1])
p.resetBasePositionAndOrientation(sphere_id, [1, 0, 1], [0, 0, 0, 1])
p.resetBasePositionAndOrientation(cylinder_id, [2, 0, 1], [0, 0, 0, 1])

# Set the box's, sphere's, pyramid's, and cylinder's mass and friction
p.createMultiBody(1, box_id, -1, [0, 0, 1], [0, 0, 0, 1])
p.createMultiBody(1, sphere_id, -1, [1, 0, 1], [0, 0, 0, 1])
p.createMultiBody(1, cylinder_id, -1, [2, 0, 1], [0, 0, 0, 1])

# Set the box's, sphere's, and cylinder's initial velocity
p.resetBaseVelocity(box_id, [0, 0, 5])
p.resetBaseVelocity(sphere_id, [0, 0, 5])
p.resetBaseVelocity(cylinder_id, [0, 0, 5])

# Run the simulation for 10 seconds
for i in range(6000):
    # Step the simulation
    p.stepSimulation()

    # Sleep for a small amount of time
    time.sleep(0.01)

# Disconnect from the physics server
p.disconnect()
