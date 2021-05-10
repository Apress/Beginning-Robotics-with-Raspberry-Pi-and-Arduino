import robot_sample_class as Robot

my_robot = Robot.Robot(
        "Nomad", "autonomous Rover",
        "Black", "Jeff Cicolani")

print("My robot is a " + my_robot.desc + " called " +
          my_robot.name)
my_robot.drive_forward()
my_robot.drive_backward()
my_robot.turn_left()
my_robot.turn_right()
my_robot.set_speed(255)
my_robot.set_duration(1000)


