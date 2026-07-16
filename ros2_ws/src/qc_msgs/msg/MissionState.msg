# Current mission phase. Published (latched) by TaskManager; read by the web
# console and the Phase-2 inspection node. See architecture.md §4.1.
#
# phase is one of:
#   idle | planning | planned | executing | scanning | inspecting |
#   complete | rescanning | aborted | error
string phase
string part_id      # the DEX part id this mission is for ("" when idle)
uint32 attempt      # scan attempt number (1-based; increments on a rescan)
string detail       # human-readable note (e.g. an error message)
