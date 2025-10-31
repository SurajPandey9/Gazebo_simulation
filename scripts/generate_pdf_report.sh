#!/bin/bash
# Generate PDF report from Markdown template with exact formatting requirements
# Requirements: Times New Roman, 12pt, 1.5 line spacing, 1 inch margins

set -e

echo "=========================================="
echo "Generating PDF Report"
echo "=========================================="
echo ""

# Check if pandoc is installed
if ! command -v pandoc &> /dev/null; then
    echo "ERROR: pandoc is not installed!"
    echo ""
    echo "Install pandoc:"
    echo "  Ubuntu/Debian: sudo apt-get install pandoc texlive-latex-base texlive-fonts-recommended texlive-latex-extra"
    echo "  macOS: brew install pandoc basictex"
    echo "  Windows: Download from https://pandoc.org/installing.html"
    echo ""
    exit 1
fi

# Input and output files
INPUT_FILE="docs/REPORT_TEMPLATE.md"
OUTPUT_FILE="UAV_Simulation_Report.pdf"

# Check if input file exists
if [ ! -f "$INPUT_FILE" ]; then
    echo "ERROR: Report template not found at $INPUT_FILE"
    exit 1
fi

echo "Input:  $INPUT_FILE"
echo "Output: $OUTPUT_FILE"
echo ""

# Create temporary LaTeX header for formatting
cat > /tmp/report_header.tex << 'EOF'
\usepackage{geometry}
\geometry{margin=1in}
\usepackage{setspace}
\onehalfspacing
\usepackage{fontspec}
\setmainfont{Times New Roman}
\usepackage{graphicx}
\usepackage{hyperref}
\hypersetup{
    colorlinks=true,
    linkcolor=blue,
    filecolor=magenta,
    urlcolor=cyan,
}
\usepackage{fancyhdr}
\pagestyle{fancy}
\fancyhf{}
\rhead{\thepage}
\lhead{UAV Target Detection and Engagement System}
\renewcommand{\headrulewidth}{0.4pt}
EOF

echo "Generating PDF with pandoc..."
echo ""

# Generate PDF with exact specifications
pandoc "$INPUT_FILE" \
    -o "$OUTPUT_FILE" \
    --pdf-engine=xelatex \
    -V fontsize=12pt \
    -V geometry:margin=1in \
    -V linestretch=1.5 \
    -V mainfont="Times New Roman" \
    --toc \
    --toc-depth=3 \
    --number-sections \
    -H /tmp/report_header.tex \
    --highlight-style=tango

# Check if PDF was created
if [ -f "$OUTPUT_FILE" ]; then
    echo "=========================================="
    echo "PDF Report Generated Successfully!"
    echo "=========================================="
    echo ""
    echo "File: $OUTPUT_FILE"
    echo "Size: $(du -h "$OUTPUT_FILE" | cut -f1)"
    echo ""
    echo "Formatting:"
    echo "  ✓ Font: Times New Roman"
    echo "  ✓ Size: 12 point"
    echo "  ✓ Line Spacing: 1.5"
    echo "  ✓ Margins: 1 inch (all sides)"
    echo "  ✓ Table of Contents: Included"
    echo "  ✓ Page Numbers: Included"
    echo ""
    echo "Next steps:"
    echo "  1. Open and review: $OUTPUT_FILE"
    echo "  2. Fill in your personal information (name, course, date)"
    echo "  3. Add your experimental results and screenshots"
    echo "  4. Add any additional analysis or discussion"
    echo "  5. Submit the PDF"
    echo ""
else
    echo "ERROR: PDF generation failed!"
    exit 1
fi

# Clean up
rm -f /tmp/report_header.tex

echo "Done!"

