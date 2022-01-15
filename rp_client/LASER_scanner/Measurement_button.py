import RPi.GPIO as GPIO


class Measurement_button:
    def __init__(self, pin_number):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin_number, GPIO.IN)
        self.pin_number = pin_number

    def __del__(self):
        GPIO.cleanup()

    def get_state(self):
        return GPIO.input(self.pin_number)


if __name__ == '__main__':
    """Test code:
      1. get the state of switch a few times
      """
    pin = Measurement_button(20)
    for i in range(10):
        print(pin.get_state())

    del pin
