#!/bin/bash
#SBATCH -J HN_EngVerify
#SBATCH -A uoguzman
#SBATCH -p orfoz,debug
#SBATCH -N 1
#SBATCH -n 4
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

cd "${WORKSPACE_DIR}"
mkdir -p logs results artifacts

echo "[TRUBA] Rebuilding proprioception_plugin & HeadNeckNeuralController_plugin on $(hostname) at $(date)"
rm -rf proprioception_plugin/build HeadNeckNeuralController_plugin/build
mkdir -p proprioception_plugin/build HeadNeckNeuralController_plugin/build
cmake -S proprioception_plugin -B proprioception_plugin/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DOpenSim_DIR="${OpenSim_DIR}" \
    -DCMAKE_PREFIX_PATH="${CONDA_PREFIX}"
cmake --build proprioception_plugin/build --config Release --target osimMillard12EqWithAff -j 4

cmake -S HeadNeckNeuralController_plugin -B HeadNeckNeuralController_plugin/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DOpenSim_DIR="${OpenSim_DIR}" \
    -DCMAKE_PREFIX_PATH="${CONDA_PREFIX}"
cmake --build HeadNeckNeuralController_plugin/build --config Release --target osimHeadNeckNeural -j 4

echo "[TRUBA] Launching 3-axis passive Bode (--tag c11_engagement_check) & RPY (+/-50 deg Yaw) sweep in parallel at $(date)"
python scripts/generate_standard_plot.py \
    --params-json logs/se2_c1pos_3axis_champion.json \
    --tag c11_engagement_check \
    > logs/truba_bode_c11_engagement_check.log 2>&1 &
PID_BODE=$!

python scripts/run_rpy_50deg_yaw_sweep.py \
    > logs/truba_rpy_50deg_yaw_sweep.log 2>&1 &
PID_RPY=$!

wait "${PID_BODE}"
RC_BODE=$?
wait "${PID_RPY}"
RC_RPY=$?
echo "[TRUBA] Completed at $(date) | RC_BODE=${RC_BODE} | RC_RPY=${RC_RPY}"
test "${RC_BODE}" -eq 0 && test "${RC_RPY}" -eq 0
