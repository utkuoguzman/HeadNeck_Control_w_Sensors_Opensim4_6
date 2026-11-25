#!/usr/bin/env bash
set -euo pipefail

echo "==================================================================="
echo " TRUBA ARF Setup & C++ Plugin Build Script (OpenSim 4.6 + AVX2/FMA)"
echo "==================================================================="

MINIFORGE_DIR="/arf/scratch/${USER}/miniforge3"
WORKSPACE_DIR="/arf/scratch/${USER}/HeadNeck_Control_w_Sensors_Opensim4_6"

# 1. Install Miniforge3 if not present
if [ ! -d "${MINIFORGE_DIR}" ]; then
    echo "[1/4] Installing Miniforge3 into ${MINIFORGE_DIR}..."
    wget -q https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh -O /tmp/Miniforge3.sh
    bash /tmp/Miniforge3.sh -b -p "${MINIFORGE_DIR}"
    rm -f /tmp/Miniforge3.sh
fi

set +u
source "${MINIFORGE_DIR}/etc/profile.d/conda.sh"

# 2. Create or activate opensim_env
if ! conda env list | grep -q "opensim_env"; then
    echo "[2/4] Creating conda environment 'opensim_env'..."
    conda create -y -n opensim_env -c opensim-org -c conda-forge \
        python=3.11 opensim numpy scipy matplotlib cma cmake compilers make
fi

conda activate opensim_env
set -u
pip install --no-deps --upgrade "cma==4.5.0"
export OpenSim_DIR="${CONDA_PREFIX}/lib/cmake/OpenSim"
export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export MPLBACKEND=Agg

cd "${WORKSPACE_DIR}"

# 3. Build the 4 C++ OpenSim plugins in dependency order
echo "[3/4] Compiling C++ OpenSim plugins (-O3 -march=native -flto)..."

for PLUGIN in vestibular_plugin proprioception_plugin HeadNeckNeuralController_plugin FixationController_plugin; do
    echo "  -> Building ${PLUGIN}..."
    rm -rf "${PLUGIN}/build"
    mkdir -p "${PLUGIN}/build"
    cmake -S "${PLUGIN}" -B "${PLUGIN}/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DOpenSim_DIR="${OpenSim_DIR}" \
        -DCMAKE_PREFIX_PATH="${CONDA_PREFIX}"
    cmake --build "${PLUGIN}/build" --config Release -j 8
done

# 4. Verify plugin loading in Python
echo "[4/4] Verifying Linux .so plugin loading & runner import in OpenSim..."
python -c "import sys; sys.path.extend(['.', 'util', 'scripts']); import opensim as osim, cmaes_worker_p22 as w; w.ensure_libraries_loaded(); m = osim.Model('osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim'); ctrls = [m.getControllerSet().get(i).getConcreteClassName() for i in range(m.getControllerSet().getSize())]; assert 'HeadNeckNeuralController' in ctrls, ctrls; import scripts.generate_standard_plot; print('[SUCCESS] All 4 Linux OpenSim plugins & HeadNeckNeuralController loaded cleanly!')"

echo "==================================================================="
echo " Setup Complete! Submit job with: sbatch truba/submit_truba_arf.slurm"
echo "==================================================================="

