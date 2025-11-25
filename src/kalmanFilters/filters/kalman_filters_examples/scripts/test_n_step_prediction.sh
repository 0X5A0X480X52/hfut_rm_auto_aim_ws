#!/bin/bash

# Test N-step prediction analysis for a specific filter
# Usage: ./test_n_step_prediction.sh [filter_name]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Default filter to test
FILTER=${1:-"cv_kf_3d"}

# Paths
TEST_DATA=""
RESULT_FILE=""
OUTPUT_DIR=""

case $FILTER in
    "cv_kf_3d")
        TEST_DATA="$PROJECT_ROOT/test_data/cv_3d.csv"
        RESULT_FILE="$PROJECT_ROOT/results/cv_kf_3d_result.csv"
        OUTPUT_DIR="$PROJECT_ROOT/analysis/cv_kf_3d"
        ;;
    "ca_kf_3d")
        TEST_DATA="$PROJECT_ROOT/test_data/ca_3d.csv"
        RESULT_FILE="$PROJECT_ROOT/results/ca_kf_3d_result.csv"
        OUTPUT_DIR="$PROJECT_ROOT/analysis/ca_kf_3d"
        ;;
    "imm_cv_ca_cs_3d")
        TEST_DATA="$PROJECT_ROOT/test_data/mixed_3d.csv"
        RESULT_FILE="$PROJECT_ROOT/results/imm_cv_ca_cs_3d_result.csv"
        OUTPUT_DIR="$PROJECT_ROOT/analysis/imm_cv_ca_cs_3d"
        ;;
    *)
        echo "Unknown filter: $FILTER"
        echo "Available filters: cv_kf_3d, ca_kf_3d, imm_cv_ca_cs_3d"
        exit 1
        ;;
esac

echo "=== N-Step Prediction Analysis ==="
echo "Filter: $FILTER"
echo "Test data: $TEST_DATA"
echo "Result file: $RESULT_FILE"
echo "Output directory: $OUTPUT_DIR"
echo ""

# Check if files exist
if [ ! -f "$TEST_DATA" ]; then
    echo "Error: Test data file not found: $TEST_DATA"
    exit 1
fi

if [ ! -f "$RESULT_FILE" ]; then
    echo "Error: Result file not found: $RESULT_FILE"
    echo "Please run the filter test first."
    exit 1
fi

# Run analysis with N-step prediction
python3 "$SCRIPT_DIR/analyze_results.py" \
    "$TEST_DATA" \
    "$RESULT_FILE" \
    --output "$OUTPUT_DIR" \
    --n-step \
    --horizons 1 5 10 20 50 100

echo ""
echo "=== Analysis Complete ==="
echo "Results saved to: $OUTPUT_DIR"
echo ""
echo "Generated files:"
echo "  - trajectory.png           : Trajectory comparison"
echo "  - errors.png               : Error analysis"
echo "  - statistics.json          : Overall statistics"
echo "  - n_step_predictions.png   : N-step prediction analysis"
echo "  - n_step_predictions.json  : N-step prediction statistics"
