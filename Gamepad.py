from inputs import get_gamepad
class Gamepad:
    """
    Simple Xbox controller reader.
    Call update() in the main loop to refresh values.
    """

    def __init__(self):
        self.left_stick_y = 0.0
        self.right_trigger = 0.0
        self.left_trigger = 0.0
        self.button_a = False
        self.button_b = False

    def update(self):
        """Read the controller inputs once and update attributes."""
        try:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':          # Left stick vertical
                    self.left_stick_y = event.state / 32767.0
                elif event.code == 'ABS_RZ':       # Right trigger
                    self.right_trigger = event.state / 255.0
                elif event.code == 'ABS_Z':        # Left trigger
                    self.left_trigger = event.state / 255.0
                elif event.code == 'BTN_SOUTH':    # A button
                    self.button_a = event.state == 1
                elif event.code == 'BTN_EAST':     # B button
                    self.button_b = event.state == 1
        except Exception as e:
            print(f"Gamepad read error: {e}")
