# Proportional control on state error
def P(currentState, referenceState):
    p = 0.01
    return p * (referenceState - currentState)