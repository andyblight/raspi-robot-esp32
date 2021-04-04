# Encoder message definition.

# There are two encoders, one for each motor.
# The values will be published every tick.
# +ve count is forward, -ve count is reverse, 0 is no change.
int16 left
int16 right
