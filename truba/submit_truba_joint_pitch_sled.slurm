#!/bin/bash
#SBATCH -J HN_FastJPS
#SBATCH -A uoguzman
#SBATCH -p orfoz
#SBATCH -N 1
#SBATCH -n 56
#SBATCH -c 1
#SBATCH --time=03:30:00
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
export OpenSim_DIR="${CONDA_PREFIX}/lib/cmake/OpenSim"
export SIMB_NUM_THREADS=1
export SIMBODY_NUM_THREADS=1
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export MPLBACKEND=Agg

cd "${WORKSPACE_DIR}"
mkdir -p logs results

echo "[TRUBA FastJPS] Starting on $(hostname) at $(date)"
test -f HeadNeckNeuralController_plugin/build/libosimHeadNeckNeural.so || {
    echo "ERROR: libosimHeadNeckNeural.so missing!"; exit 1;
}

python scripts/truba_joint_pitch_sled_cmaes.py \
    --phase cmaes \
    --pop 26 \
    --gens 40 \
    --workers 56 \
    > logs/truba_fast_joint_pitch_sled_cmaes.log 2>&1

echo "[TRUBA FastJPS] Finished at $(date)"
