#!/bin/bash
# Demo script for N-step prediction analysis
# 演示N步预测分析功能

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "======================================================================"
echo "N-Step Prediction Analysis Demo"
echo "演示卡尔曼滤波器N步预测准确率分析"
echo "======================================================================"
echo ""

# Example 1: CV Filter with short-term predictions
echo "Example 1: CV Filter - Short-term predictions (1-10 steps)"
echo "----------------------------------------------------------------------"
python3 "$SCRIPT_DIR/analyze_results.py" \
    "$PROJECT_ROOT/test_data/cv_3d.csv" \
    "$PROJECT_ROOT/results/cv_kf_3d_result.csv" \
    --output "$PROJECT_ROOT/analysis/cv_kf_3d_demo" \
    --n-step \
    --horizons 1 2 3 5 10

echo ""
echo "Results saved to: $PROJECT_ROOT/analysis/cv_kf_3d_demo/"
echo "  - n_step_predictions.png: Visualization"
echo "  - n_step_predictions.json: Detailed statistics"
echo ""
echo "Press Enter to continue..."
read

# Example 2: CA Filter with medium-term predictions
echo ""
echo "Example 2: CA Filter - Medium-term predictions (1-20 steps)"
echo "----------------------------------------------------------------------"
python3 "$SCRIPT_DIR/analyze_results.py" \
    "$PROJECT_ROOT/test_data/ca_3d.csv" \
    "$PROJECT_ROOT/results/ca_kf_3d_result.csv" \
    --output "$PROJECT_ROOT/analysis/ca_kf_3d_demo" \
    --n-step \
    --horizons 1 5 10 15 20

echo ""
echo "Results saved to: $PROJECT_ROOT/analysis/ca_kf_3d_demo/"
echo ""
echo "Press Enter to continue..."
read

# Example 3: IMM Filter with long-term predictions
echo ""
echo "Example 3: IMM Filter - Long-term predictions (1-100 steps)"
echo "----------------------------------------------------------------------"
echo "Note: IMM has no velocity estimates, so long-term prediction accuracy"
echo "      will degrade quickly as it uses constant position model."
echo ""
python3 "$SCRIPT_DIR/analyze_results.py" \
    "$PROJECT_ROOT/test_data/mixed_3d.csv" \
    "$PROJECT_ROOT/results/imm_cv_ca_cs_3d_result.csv" \
    --output "$PROJECT_ROOT/analysis/imm_cv_ca_cs_3d_demo" \
    --n-step \
    --horizons 1 5 10 20 50 100

echo ""
echo "Results saved to: $PROJECT_ROOT/analysis/imm_cv_ca_cs_3d_demo/"
echo ""

# Summary
echo ""
echo "======================================================================"
echo "Demo Complete!"
echo "======================================================================"
echo ""
echo "Key Observations:"
echo ""
echo "1. CV and CA filters use velocity for prediction"
echo "   → Better long-term prediction accuracy"
echo "   → RMSE grows linearly with prediction horizon"
echo ""
echo "2. IMM filter has no velocity estimates"
echo "   → Uses constant position assumption"
echo "   → RMSE grows rapidly for longer horizons"
echo ""
echo "3. Typical RMSE values:"
echo "   - 1-step:   0.06-0.07 m  (very accurate)"
echo "   - 10-step:  0.10-0.18 m  (good for short-term)"
echo "   - 20-step:  0.14-0.35 m  (depends on filter type)"
echo "   - 50-step:  0.30-0.85 m  (only CV/CA reliable)"
echo ""
echo "For more details, see: $PROJECT_ROOT/N_STEP_PREDICTION.md"
echo "======================================================================"
