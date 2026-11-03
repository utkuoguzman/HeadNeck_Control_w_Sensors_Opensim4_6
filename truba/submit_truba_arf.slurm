#!/bin/bash
#SBATCH -J HeadNeck_P24_CMAES
#SBATCH -A <YOUR_TRUBA_ACCOUNT>
#SBATCH -p hamsi
#SBATCH -N 1
#SBATCH -n 1
#SBATCH --cpus-per-task=56
#SBATCH --time=72:00:00
#SBATCH --output=/arf/scratch/%u/HeadNeck_Control_w_Sensors_Opensim4_6/logs/slurm-%j.out
#SBATCH --error=/arf/scratch/%u/HeadNeck_Control_w_Sensors_Opensim4_6/logs/slurm-%j.err

set -euo pipefail

MINIFORGE_DIR="/arf/home/${USER}/miniforge3"
WORKSPACE_DIR="/arf/scratch/${USER}/HeadNeck_Control_w_Sensors_Opensim4_6"

source "${MINIFORGE_DIR}/etc/profile.d/conda.sh"
conda activate opensim_env

export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${WORKSPACE_DIR}/vestibular_plugin/build:${WORKSPACE_DIR}/proprioception_plugin/build:${WORKSPACE_DIR}/HeadNeckNeuralController_plugin/build:${WORKSPACE_DIR}/FixationController_plugin/build:${LD_LIBRARY_PATH:-}"
export SIMB_NUM_THREADS=1
export OMP_NUM_THREADS=1
export DISABLE_NIGHT_PAUSE=1
export PHASE2_4_WORKERS="${SLURM_CPUS_PER_TASK:-56}"
export PHASE2_4_MAX_GEN=80
export PHASE2_4_STOP_GEN=80

cd "${WORKSPACE_DIR}"
echo "[TRUBA ARF] Starting Phase 2.4 CMA-ES on $(hostname) with ${PHASE2_4_WORKERS} physical cores at $(date)"
python scripts/run_phase2_4_campaign.py
echo "[TRUBA ARF] Finished Phase 2.4 CMA-ES at $(date)"

