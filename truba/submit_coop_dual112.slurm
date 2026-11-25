#!/bin/bash
#SBATCH -J HN_Dual112
#SBATCH -A uoguzman
#SBATCH -p orfoz
#SBATCH -N 1
#SBATCH -n 112
#SBATCH -c 1
#SBATCH --time=08:00:00
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
python scripts/run_cooperative_dual_18p_roll_pitch_cmaes.py --mode dual --pop-roll 72 --pop-pitch 36 --gens 160 --workers 112 --resume
echo "[TRUBA Coop Dual112] Finished at $(date)"
