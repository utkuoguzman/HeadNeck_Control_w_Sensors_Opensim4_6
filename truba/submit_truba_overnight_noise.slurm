#!/bin/bash
#SBATCH -J HN_NoiseMC
#SBATCH -A uoguzman
#SBATCH -p orfoz
#SBATCH -N 1
#SBATCH -n 56
#SBATCH -c 1
#SBATCH --time=02:00:00
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
export HN_CHAMP_JSON="${HN_CHAMP_JSON:-${WORKSPACE_DIR}/logs/se2_c1pos_3axis_champion.json}"

cd "${WORKSPACE_DIR}"
mkdir -p logs results logs/joint_verify_7050190 results/joint_verify_7050190

# Archive 7050190 joint-verify artifacts & restore 7049173 verified baseline files cleanly
for f in logs/rpy_50deg_yaw_sweep.json logs/validation_5dof_voluntary_shift.json logs/zheng_2g_3g_urgency_sweep.json; do
    [ -f "$f" ] && cp -f "$f" logs/joint_verify_7050190/
done
for f in results/rpy_50deg_yaw_sweep.png results/validation_5dof_voluntary_shift.png results/zheng_2g_3g_urgency_sweep.png; do
    [ -f "$f" ] && cp -f "$f" results/joint_verify_7050190/
done
if [ -d logs/backup_7049173 ]; then
    cp -f logs/backup_7049173/*.json logs/ 2>/dev/null || true
fi
if [ -d results/backup_7049173 ]; then
    cp -f results/backup_7049173/*.png results/ 2>/dev/null || true
fi

echo "[TRUBA NoiseMC] Starting on $(hostname) at $(date) with HN_CHAMP_JSON=${HN_CHAMP_JSON}"
python scripts/truba_overnight_noise_suite.py > logs/truba_overnight_noise_suite.log 2>&1
echo "[TRUBA NoiseMC] Finished at $(date)"
