# Proportional control on state error
def P(currentState, referenceState):
    p = 5.0
    return p * (referenceState - currentState)