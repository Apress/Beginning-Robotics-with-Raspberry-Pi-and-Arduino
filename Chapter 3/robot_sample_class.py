class Robot():
    """
    a simple robot class
    This multi-line comment is a good place
    to provide a description of what the class
    is.
    """

    # define the initiating function.
    # speed = value between 0 and 255
    # duration = value in milliseconds
    def __init__(self, name, desc, color, owner,
                 speed = 125, duration = 100):
        # initialize our robot
        self.name = name
        self.desc = desc
        self.color = color
        self.owner = owner
        self.speed = speed
        self.duration = duration

    def drive_forward(self):
        # simulates driving the robot forward
        print(self.name.title() + " is driving" +
              " forward " + str(self.duration) +
              " milliseconds")

    def drive_backward(self):
        # simulates driving the robot backward
        print(self.name.title() + " is driving" +
              " backward " + str(self.duration) +
              " milliseconds")

    def turn_left(self):
        # simulates turning left
        print(self.name.title() + " is turning" +
              " left " + str(self.duration) +
              " milliseconds")

    def turn_right(self):
        # simulates turning right
        print(self.name.title() + " is turning" +
              " right " + str(self.duration) +
              " milliseconds")

    def set_speed(self, speed):
        # sets the speed of the motors
        self.speed = speed
        print("the motor speed is now " +
              str(self.speed))

    def set_duration(self, duration):
        # sets the duration of travel
        self.duration = duration
        print("the duration is now " +
              str(self.duration))

