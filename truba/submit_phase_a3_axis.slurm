#!/bin/bash
#SBATCH -J A3_Axis
#SBATCH -A uoguzman
#SBATCH -p orfoz
#SBATCH -N 1
#SBATCH -n 40
#SBATCH -c 1
#SBATCH --time=07:15:00
#SBATCH --chdir=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6
#SBATCH --output=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/logs/phase_a3/slurm-%x-%j.out
#SBATCH --error=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/logs/phase_a3/slurm-%x-%j.err

set -euo pipefail
MINIFORGE_DIR="/arf/scratch/${USER}/miniforge3"
WORKSPACE_DIR="/arf/scratch/${USER}/HeadNeck_Control_w_Sensors_Opensim4_6"
set +u
source "${MINIFORGE_DIR}/etc/profile.d/conda.sh"
conda activate opensim_env
set -u

export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${WORKSPACE_DIR}/vestibular_plugin/build:${WORKSPACE_DIR}/proprioception_plugin/build:${WORKSPACE_DIR}/HeadNeckNeuralController_plugin/build:${WORKSPACE_DIR}/FixationController_plugin/build:${LD_LIBRARY_PATH:-}"
export SIMB_NUM_THREADS=1 SIMBODY_NUM_THREADS=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 MPLBACKEND=Agg

cd "${WORKSPACE_DIR}"
mkdir -p logs/phase_a3
STOP_EPOCH=$(date -d "2026-11-25 06:20:00" +%s)
echo "[PHASE-A3 ${A3_MODE:-roll}] host=$(hostname) ntasks=${SLURM_NTASKS} island=${A3_ISLAND:-main} stop_epoch=${STOP_EPOCH} at $(date)"

python -u scripts/truba_phase_a3_joint_cmaes.py \
    --mode "${A3_MODE:-roll}" \
    --island "${A3_ISLAND:-main}" \
    --pop "${A3_POP:-38}" \
    --gens "${A3_GENS:-140}" \
    --sigma "${A3_SIGMA:-0.40}" \
    --seed "${A3_SEED:-20261129}" \
    --workers "${SLURM_NTASKS}" \
    --stop-epoch "${STOP_EPOCH}"

echo "[PHASE-A3 ${A3_MODE:-roll}] Finished at $(date)"
