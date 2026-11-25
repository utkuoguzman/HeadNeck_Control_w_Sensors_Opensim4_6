#!/bin/bash
#SBATCH -J HN_LatCln
#SBATCH -A uoguzman
#SBATCH -p debug,orfoz
#SBATCH -N 1
#SBATCH -n 112
#SBATCH -c 1
#SBATCH --time=04:00:00
#SBATCH --chdir=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6
#SBATCH --output=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/logs/slurm-%j.out
#SBATCH --error=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/logs/slurm-%j.err

set -euo pipefail

MINIFORGE_DIR="/arf/scratch/${USER}/miniforge3"
WORKSPACE_DIR="/arf/scratch/${USER}/HeadNeck_Control_w_Sensors_Opensim4_6"

set +u
source "${MINIFORGE_DIR}/etc/profile.d/conda.sh"
conda activate opensim_env
set -u

export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${WORKSPACE_DIR}/vestibular_plugin/build:${WORKSPACE_DIR}/proprioception_plugin/build:${WORKSPACE_DIR}/HeadNeckNeuralController_plugin/build:${WORKSPACE_DIR}/FixationController_plugin/build:${LD_LIBRARY_PATH:-}"
export SIMB_NUM_THREADS=1
export SIMBODY_NUM_THREADS=1
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export MPLBACKEND=Agg

cd "${WORKSPACE_DIR}"
if [ ! -f "HeadNeckNeuralController_plugin/build/libosimHeadNeckNeural.so" ]; then
    for PLUGIN in vestibular_plugin proprioception_plugin HeadNeckNeuralController_plugin FixationController_plugin; do
        rm -rf "${PLUGIN}/build"
        mkdir -p "${PLUGIN}/build"
        cmake -S "${PLUGIN}" -B "${PLUGIN}/build" \
            -DCMAKE_BUILD_TYPE=Release \
            -DOpenSim_DIR="${OpenSim_DIR:-${CONDA_PREFIX}/lib/cmake/OpenSim}" \
            -DCMAKE_PREFIX_PATH="${CONDA_PREFIX}"
        cmake --build "${PLUGIN}/build" --config Release -j 8
    done
fi

python scripts/run_latec_only_18p_roll_clean_cmaes.py --gens 80 --workers 112 --resume
echo "[TRUBA ARF] Finished at $(date)"
