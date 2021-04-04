# Encoder message definition.

# There are two encoders, one for each motor.
# The values are the count of encoder pules since the last message.
# There are 20 encoder pulses per revolution of the wheel.
# The message will be published every on tick.
# +ve count is forward, -ve count is reverse, 0 is no change.
int16 left
int16 right
