from controller import Robot

# create the Robot instance
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# get the motor
base_motor = robot.getMotor("base")
base_motor.setPosition(0.1)  # initial position
base_motor.setVelocity(0.7)  # speed

# test angle positions
angles = [0.7, -0.8, 1.0, -1.0, 0.0]  # radians
index = 0
duration = 1  # steps to wait at each position

counter = 0

while robot.step(timestep) != -1:
    if counter % duration == 0:
        # move motor to next angle
        base_motor.setPosition(angles[index])
        index = (index + 1) % len(angles)
    counter += 1
