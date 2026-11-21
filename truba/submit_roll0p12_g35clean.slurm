#!/bin/bash
#SBATCH -J HN_R012_G35
#SBATCH -A uoguzman
#SBATCH -p orfoz,debug
#SBATCH -N 1
#SBATCH -n 40
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
python scripts/run_roll_0p12_phase_cmaes.py --run-tag g35clean --seed-preset g35clean --pop-roll 36 --gens 150 --workers 40 --seed 6202 --resume
echo "[TRUBA Roll0p12 g35clean] Finished at $(date)"
