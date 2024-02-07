''' Python module that is used for the example
    Buildings.Utilities.IO.Python36.Examples.Kalman
'''
def constant(dblInp, state):
    ''' an example of adding constant temperature difference
        on waterflow leaving ground
    '''
    [T0, Q_flow, tim] = dblInp
    if state == None:
        # Initialize the state
        state = {'tLast': tim, 'T': T0, 'Q': Q_flow}
    else:
        # Use the python object
        dt = tim - state['tLast']
        T = T0 + 0.5
        state = {'tLast': tim, 'T': T, 'Q': Q_flow}
    return [[state['T'], state['Q']], state]
