#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
cmake -S . -B build
cmake --build build -j
./build/servo20 --scenario nominal --realtime --out results
for scenario in jam controller_heartbeat drive_heartbeat; do
  ./build/servo20 --scenario "$scenario" --out results
done
python3 tools/plot_results.py --input results --output plots
python3 tools/check_results.py
printf '\nDone. CSV: results/    Figures: plots/\n'
