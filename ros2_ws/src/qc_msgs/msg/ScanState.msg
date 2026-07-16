# ScanningDriver capture state, published on /scan/state.
#
# state is one of:  idle | scanning | done | error
string state
string cloud_path   # exported point-cloud file path (set when state == done)
string detail       # human-readable note (e.g. an error message)
