#!/bin/bash
#SBATCH -J HN_JointVer
#SBATCH -A uoguzman
#SBATCH -p orfoz
#SBATCH -N 1
#SBATCH -n 56
#SBATCH -c 1
#SBATCH --time=00:45:00
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
export HN_CHAMP_JSON="${WORKSPACE_DIR}/logs/truba_joint_pitch_sled_champion.json"

cd "${WORKSPACE_DIR}"
mkdir -p logs results artifacts logs/backup_7049173 results/backup_7049173
for f in logs/rpy_50deg_yaw_sweep.json logs/validation_5dof_voluntary_shift.json logs/zheng_2g_3g_urgency_sweep.json; do
    [ -f "$f" ] && cp -f "$f" logs/backup_7049173/
done
for f in results/rpy_50deg_yaw_sweep.png results/validation_5dof_voluntary_shift.png results/zheng_2g_3g_urgency_sweep.png; do
    [ -f "$f" ] && cp -f "$f" results/backup_7049173/
done

echo "[TRUBA JointVer] Starting 4 suites on $(hostname) at $(date) with HN_CHAMP_JSON=${HN_CHAMP_JSON}"
python scripts/generate_standard_plot.py \
    --params-json "${HN_CHAMP_JSON}" \
    --tag c11_joint_sled_bode \
    > logs/truba_bode_c11_joint_sled_bode.log 2>&1 &
PID_BODE=$!

python scripts/run_rpy_50deg_yaw_sweep.py \
    > logs/truba_joint_rpy_50deg_yaw_sweep.log 2>&1 &
PID_RPY=$!

python scripts/validate_5dof_voluntary_and_se2_shift.py \
    > logs/truba_joint_5dof_voluntary_shift.log 2>&1 &
PID_5DOF=$!

python scripts/run_zheng_2g_3g_urgency_sweep.py \
    > logs/truba_joint_zheng_2g_3g_urgency_sweep.log 2>&1 &
PID_SLED=$!

set +e
wait "${PID_BODE}"; RC_BODE=$?
wait "${PID_RPY}";  RC_RPY=$?
wait "${PID_5DOF}"; RC_5DOF=$?
wait "${PID_SLED}"; RC_SLED=$?
set -e

echo "[TRUBA JointVer] Completed at $(date) | RC_BODE=${RC_BODE} | RC_RPY=${RC_RPY} | RC_5DOF=${RC_5DOF} | RC_SLED=${RC_SLED}"
test "${RC_BODE}" -eq 0 && test "${RC_RPY}" -eq 0 && test "${RC_5DOF}" -eq 0 && test "${RC_SLED}" -eq 0
