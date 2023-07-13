import machine

# Define the pins
PIN_DIFF = machine.Pin(4, machine.Pin.IN)
PIN_TRICYCLE = machine.Pin(5, machine.Pin.IN)
PIN_DOUBLE = machine.Pin(6, machine.Pin.IN)

# Check the state of the pins to determine which code file to run
if PIN_DIFF.value() == 1:
    import main_differential  # Run differential if pin 6 is connected to 3.3V 
elif PIN_TRICYCLE.value() == 1:
    import main_tricycle  # Run tricycle if pin 5 is connected to 3.3V
elif PIN_DOUBLE.value() == 1:
    import main_double # Run double tricycle if pin 6 is connected to 3.3V
