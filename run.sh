#!/usr/bin/env bash
# Launcher for the ME5415 food-tray assembly scene.
# Sets LD_LIBRARY_PATH so SofaPython3 finds libpython3.12 from the
# conda environment, then opens the scene in runSofa.

set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOFA_ROOT="/home/kirito/sofa-install/extracted/SOFA_v24.12.00_Linux"
PYLIB_DIR="/home/kirito/anaconda3/envs/sofa/lib"

export LD_LIBRARY_PATH="${PYLIB_DIR}:${LD_LIBRARY_PATH}"

cd "${HERE}"
exec "${SOFA_ROOT}/bin/runSofa" -g qt -l SofaPython3 "${HERE}/scene.py" "$@"
