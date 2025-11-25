#!/bin/bash
# Run all Kalman filter tests
# 运行所有卡尔曼滤波器测试

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"

# Locate or prepare filter_test executable
FILTER_TEST_CMD=""

# Try to use a filter_test on PATH first
if command -v filter_test >/dev/null 2>&1; then
    FILTER_TEST_CMD=$(command -v filter_test)
else
    # Try to source a workspace install/setup.bash (if any) going upwards
    SEARCH_DIR="${PACKAGE_DIR}"
    while [ "$SEARCH_DIR" != "/" ]; do
        if [ -f "${SEARCH_DIR}/install/setup.bash" ]; then
            # shellcheck disable=SC1090
            source "${SEARCH_DIR}/install/setup.bash"
            break
        fi
        SEARCH_DIR="$(dirname "$SEARCH_DIR")"
    done

    # If still not found, try to locate the built binary under any parent build/ folder
    if command -v filter_test >/dev/null 2>&1; then
        FILTER_TEST_CMD=$(command -v filter_test)
    else
        SEARCH_DIR="${PACKAGE_DIR}"
        while [ "$SEARCH_DIR" != "/" ]; do
            if [ -x "${SEARCH_DIR}/build/kalman_filters_examples/filter_test" ]; then
                FILTER_TEST_CMD="${SEARCH_DIR}/build/kalman_filters_examples/filter_test"
                break
            fi
            if [ -x "${SEARCH_DIR}/install/lib/kalman_filters_examples/filter_test" ]; then
                FILTER_TEST_CMD="${SEARCH_DIR}/install/lib/kalman_filters_examples/filter_test"
                break
            fi
            SEARCH_DIR="$(dirname "$SEARCH_DIR")"
        done
    fi

    # If we still don't have it, fail early with instructions
    if [ -z "$FILTER_TEST_CMD" ]; then
        echo -e "${RED}filter_test not found in PATH or workspace build/install.\nPlease: (1) build and install the package or (2) source your workspace: source <workspace>/install/setup.bash${NC}"
        exit 1
    fi
fi

echo "Using filter_test: $FILTER_TEST_CMD"

# Locate model libraries for LD_PRELOAD to ensure factory registrations
MODEL_LIBS=""
WORKSPACE_ROOT=""

# If we're using a binary from a workspace build dir, find the workspace root
if [[ "$FILTER_TEST_CMD" == */build/* ]]; then
    WORKSPACE_ROOT="${FILTER_TEST_CMD%%/build/*}"
elif [[ "$FILTER_TEST_CMD" == */install/* ]]; then
    WORKSPACE_ROOT="${FILTER_TEST_CMD%%/install/*}"
else
    # Try to find workspace root by searching upwards
    SEARCH_DIR="${PACKAGE_DIR}"
    while [ "$SEARCH_DIR" != "/" ]; do
        if [ -d "${SEARCH_DIR}/install" ]; then
            WORKSPACE_ROOT="$SEARCH_DIR"
            break
        fi
        SEARCH_DIR="$(dirname "$SEARCH_DIR")"
    done
fi

# Build LD_PRELOAD list to force load model libraries (needed for factory registration)
if [ -n "$WORKSPACE_ROOT" ]; then
    echo "Using workspace root: ${WORKSPACE_ROOT}"
    
    # Check install directory first
    if [ -f "${WORKSPACE_ROOT}/install/basic_models/lib/libbasic_models.so" ]; then
        MODEL_LIBS="${WORKSPACE_ROOT}/install/basic_models/lib/libbasic_models.so"
    elif [ -f "${WORKSPACE_ROOT}/build/basic_models/libbasic_models.so" ]; then
        MODEL_LIBS="${WORKSPACE_ROOT}/build/basic_models/libbasic_models.so"
    fi
    
    if [ -f "${WORKSPACE_ROOT}/install/combined_models/lib/libcombined_models.so" ]; then
        if [ -n "$MODEL_LIBS" ]; then
            MODEL_LIBS="${MODEL_LIBS}:${WORKSPACE_ROOT}/install/combined_models/lib/libcombined_models.so"
        else
            MODEL_LIBS="${WORKSPACE_ROOT}/install/combined_models/lib/libcombined_models.so"
        fi
    elif [ -f "${WORKSPACE_ROOT}/build/combined_models/libcombined_models.so" ]; then
        if [ -n "$MODEL_LIBS" ]; then
            MODEL_LIBS="${MODEL_LIBS}:${WORKSPACE_ROOT}/build/combined_models/libcombined_models.so"
        else
            MODEL_LIBS="${WORKSPACE_ROOT}/build/combined_models/libcombined_models.so"
        fi
    fi
    
    if [ -n "$MODEL_LIBS" ]; then
        export LD_PRELOAD="$MODEL_LIBS"
        echo "Set LD_PRELOAD for model libraries"
    else
        echo -e "${YELLOW}Warning: Could not find model libraries for preload${NC}"
    fi
    
    # Also update LD_LIBRARY_PATH for build libs
    if [[ "$FILTER_TEST_CMD" == */build/* ]]; then
        for D in "${WORKSPACE_ROOT}/build"/*; do
            if [ -d "$D" ]; then
                shopt -s nullglob
                so_files=("$D"/*.so)
                shopt -u nullglob
                if [ ${#so_files[@]} -gt 0 ]; then
                    export LD_LIBRARY_PATH="$D:${LD_LIBRARY_PATH}"
                fi
            fi
        done
        echo "Updated LD_LIBRARY_PATH for build libs"
    fi
fi

# Default directories
CONFIG_DIR="${PACKAGE_DIR}/config"
TEST_DATA_DIR="${PACKAGE_DIR}/test_data"
RESULTS_DIR="${PACKAGE_DIR}/results"
ANALYSIS_DIR="${PACKAGE_DIR}/analysis"

# Create directories
mkdir -p "${TEST_DATA_DIR}"
mkdir -p "${RESULTS_DIR}"
mkdir -p "${ANALYSIS_DIR}"

echo "==================================================================="
echo "Kalman Filter Test Suite"
echo "==================================================================="
echo ""

# Step 1: Generate test data
echo -e "${YELLOW}Step 1: Generating test data...${NC}"
python3 "${SCRIPT_DIR}/generate_test_data.py" \
    --output-dir "${TEST_DATA_DIR}" \
    --duration 10.0 \
    --dt 0.01 \
    --noise 0.1

if [ $? -ne 0 ]; then
    echo -e "${RED}Failed to generate test data${NC}"
    exit 1
fi
echo ""

# Step 2: Run filter tests
echo -e "${YELLOW}Step 2: Running filter tests...${NC}"
echo ""

# Define test cases: "config_file|test_data_file|output_file|analysis_dir"
TESTS=(
    "cv_kf_2d.yaml|cv_2d.csv|cv_kf_2d_result.csv|cv_kf_2d"
    "cv_kf_3d.yaml|cv_3d.csv|cv_kf_3d_result.csv|cv_kf_3d"
    "ca_kf_2d.yaml|ca_2d.csv|ca_kf_2d_result.csv|ca_kf_2d"
    "ca_kf_3d.yaml|ca_3d.csv|ca_kf_3d_result.csv|ca_kf_3d"
    "cs_kf_3d.yaml|ca_3d.csv|cs_kf_3d_result.csv|cs_kf_3d"
    "singer_kf_3d.yaml|ca_3d.csv|singer_kf_3d_result.csv|singer_kf_3d"
    "ctrv_ekf_2d.yaml|circular_2d.csv|ctrv_ekf_2d_result.csv|ctrv_ekf_2d"
    "imm_cv_ca_cs_3d.yaml|mixed_3d.csv|imm_cv_ca_cs_3d_result.csv|imm_cv_ca_cs_3d"
)

SUCCESS_COUNT=0
FAIL_COUNT=0

for test in "${TESTS[@]}"; do
    IFS='|' read -r config_file test_data result_file analysis_dir <<< "$test"
    
    CONFIG_PATH="${CONFIG_DIR}/${config_file}"
    TEST_DATA_PATH="${TEST_DATA_DIR}/${test_data}"
    RESULT_PATH="${RESULTS_DIR}/${result_file}"
    ANALYSIS_PATH="${ANALYSIS_DIR}/${analysis_dir}"
    
    echo "-------------------------------------------------------------------"
    echo "Test: ${config_file%.yaml}"
    echo "  Config: ${config_file}"
    echo "  Data:   ${test_data}"
    echo "  Output: ${result_file}"
    
    # Check if config and test data exist
    if [ ! -f "${CONFIG_PATH}" ]; then
        echo -e "${RED}  Config file not found: ${CONFIG_PATH}${NC}"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        continue
    fi
    
    if [ ! -f "${TEST_DATA_PATH}" ]; then
        echo -e "${RED}  Test data file not found: ${TEST_DATA_PATH}${NC}"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        continue
    fi
    
    # Run filter test
    "$FILTER_TEST_CMD" "${CONFIG_PATH}" "${TEST_DATA_PATH}" "${RESULT_PATH}"
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}  Filter test FAILED${NC}"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        continue
    fi
    
    echo -e "${GREEN}  Filter test completed${NC}"
    
    # Run analysis
    mkdir -p "${ANALYSIS_PATH}"
    python3 "${SCRIPT_DIR}/analyze_results.py" \
        "${TEST_DATA_PATH}" \
        "${RESULT_PATH}" \
        --output "${ANALYSIS_PATH}"
    
    if [ $? -ne 0 ]; then
        echo -e "${YELLOW}  Analysis FAILED (test still passed)${NC}"
    else
        echo -e "${GREEN}  Analysis completed${NC}"
    fi
    
    SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
    echo ""
done

# Summary
echo "==================================================================="
echo "Test Summary"
echo "==================================================================="
echo "Total tests: $((SUCCESS_COUNT + FAIL_COUNT))"
echo -e "${GREEN}Passed: ${SUCCESS_COUNT}${NC}"
if [ ${FAIL_COUNT} -gt 0 ]; then
    echo -e "${RED}Failed: ${FAIL_COUNT}${NC}"
else
    echo "Failed: ${FAIL_COUNT}"
fi
echo ""
echo "Results saved to: ${RESULTS_DIR}"
echo "Analysis saved to: ${ANALYSIS_DIR}"
echo "==================================================================="

# Exit with error if any tests failed
if [ ${FAIL_COUNT} -gt 0 ]; then
    exit 1
fi
