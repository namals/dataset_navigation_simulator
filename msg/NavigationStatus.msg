uint8 status          # status of the navigator

uint8 WAITING   = 0   # Waiting for next plan
uint8 ACTIVE    = 1   # plan is being executed
uint8 SUCCEEDED = 3   # plan execution just succeeded
uint8 ABORTED   = 4   # plan execution aborted
uint8 INIT      = 5   # initial state of navigator