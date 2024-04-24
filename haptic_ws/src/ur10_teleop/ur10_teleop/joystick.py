import pygame

def initialize_joystick():
    pygame.init()
    pygame.joystick.init()  # Initialize the Joystick
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        # No joysticks!
        print("Error, I didn't find any joysticks.")
        return None
    else:
        # Use the first joystick
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Initialized Joystick : {joystick.get_name()}")
    return joystick

def print_joystick_axes(joystick, axis):
    # Assuming axis 0 & 1 are for the left joystick, 3 & 4 for the right joystick,
    # and 2 & 5 might be the triggers on some controllers.
    if axis == 0 or axis == 1:
        print(f"Left Joystick moved {'horizontally' if axis == 0 else 'vertically'} with value {joystick.get_axis(axis)}")
    elif axis == 3 or axis == 4:
        print(f"Right Joystick moved {'horizontally' if axis == 3 else 'vertically'} with value {joystick.get_axis(axis)}")
    elif axis == 2:
        print(f"LT pressed with value {joystick.get_axis(axis)}")
    elif axis == 5:
        print(f"RT pressed with value {joystick.get_axis(axis)}")

def print_hat_direction(hat_value):
    x, y = hat_value
    if x == -1:
        print("D-Pad Left")
    elif x == 1:
        print("D-Pad Right")
    if y == -1:
        print("D-Pad Down")
    elif y == 1:
        print("D-Pad Up")

def main():
    joystick = initialize_joystick()
    if not joystick:
        return

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.JOYBUTTONDOWN:
                    print(f"Button {event.button} pressed.")
                elif event.type == pygame.JOYAXISMOTION:
                    print_joystick_axes(joystick, event.axis)
                elif event.type == pygame.JOYHATMOTION:
                    print_hat_direction(joystick.get_hat(event.hat))

                print(joystick.get_hat(0)[0])
    except KeyboardInterrupt:
        print("Program exited by user")

if __name__ == "__main__":
    main()

"""
event.type ==  pygame.JOYBUTTONDOWN
event.button:
A - Button 0
B - Button 1
X - Button 2
Y - Button 3
LB - Button 4
RB - Button 5
Back - Button 6
Start - Button 7
Logitech - Button 8

event.type == pygame.JOYAXISMOTION:
axis == 0: Left joystick horizontal
axis == 1: Left joystick vertical
axis == 3: Right joystick horizontal
axis == 4: Right joystick vertical
axis == 2: LT
axis == 5: RT

event.type == pygame.JOYHATMOTION:
joystick.get_hat(event.hat)
x, y = hat_value
x == -1: D-Pad Left
x == 1: D-Pad Left
y == -1: D-Pad Left
y == 1: D-Pad Right
"""