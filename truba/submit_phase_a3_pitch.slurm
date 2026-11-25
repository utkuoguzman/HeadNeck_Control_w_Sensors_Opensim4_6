#!/bin/bash
#SBATCH -J V3_P24_Pitch
#SBATCH -A uoguzman
#SBATCH -p debug
#SBATCH -N 1
#SBATCH -n 56
#SBATCH -c 1
#SBATCH --time=04:00:00
#SBATCH --chdir=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6
#SBATCH --output=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/logs/phase_a3_v3/slurm-%x-%j.out
#SBATCH --error=/arf/scratch/uoguzman/HeadNeck_Control_w_Sensors_Opensim4_6/logs/phase_a3_v3/slurm-%x-%j.err

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
mkdir -p logs/phase_a3_v3
echo "[PHASE-A3-V3 PITCH] host=$(hostname) ntasks=${SLURM_NTASKS} island=${A3_ISLAND:-main} preset=${A3_PRESET:-default} at $(date)"

python -u scripts/truba_phase_a3_exact_roi_cmaes.py \
    --mode pitch \
    --island "${A3_ISLAND:-main}" \
    --init-preset "${A3_PRESET:-default}" \
    --pop "${A3_POP:-28}" \
    --gens "${A3_GENS:-45}" \
    --sigma "${A3_SIGMA:-0.35}" \
    --seed "${A3_SEED:-20261161}" \
    --workers "${SLURM_NTASKS}" \
    ${A3_EXTRA_ARGS:-}

echo "[PHASE-A3-V3 PITCH] Finished at $(date)"
