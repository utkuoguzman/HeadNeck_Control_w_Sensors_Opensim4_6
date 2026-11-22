#!/bin/bash
#SBATCH -J HN_C1Shd
#SBATCH -A uoguzman
#SBATCH -p debug
#SBATCH -x akya[1-10],barbun[1-40,120-130],hamsi[1-40]
#SBATCH --array=0-4
#SBATCH -N 1
#SBATCH -n 16
#SBATCH -c 1
#SBATCH --time=00:25:00
#SBATCH --chdir=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6
#SBATCH --output=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/logs/slurm-%A_%a.out
#SBATCH --error=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/logs/slurm-%A_%a.err

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
mkdir -p logs results artifacts

echo "[TRUBA] Starting C1-Pos joint shard ${SLURM_ARRAY_TASK_ID} at $(date) on $(hostname)"
python scripts/run_c1pos_joint_final_truba.py --shard-idx "${SLURM_ARRAY_TASK_ID}"
echo "[TRUBA] Finished shard ${SLURM_ARRAY_TASK_ID} at $(date)"
