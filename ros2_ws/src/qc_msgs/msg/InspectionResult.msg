# Phase-2 inspection outcome for one captured scan (architecture.md §4.1).
# The AUTOMATIC pass/fail is scan-QUALITY only (decision 4); the part verdict is
# a human call. A quality fail sets rescan_requested and TaskManager re-runs
# (up to max_rescans).
bool quality_pass
bool rescan_requested
uint32 attempt
string report_path       # deviation report file
float32 mean_dev_mm      # mean deviation vs CAD
float32 rmse_mm          # RMS deviation vs CAD
float32 coverage_pct     # fraction of the surface actually captured
