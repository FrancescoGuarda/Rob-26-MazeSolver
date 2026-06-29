#!/usr/bin/env bash

# Clean Python Cache Files Script
# Usage: ./scripts/clean.sh

echo "Cleaning Python cache files and directories..."
echo ""

# Count files before cleaning
PYCACHE_COUNT=$(find . -type d -name "__pycache__" 2>/dev/null | wc -l | tr -d ' ')
PYC_COUNT=$(find . -type f -name "*.pyc" 2>/dev/null | wc -l | tr -d ' ')
PYO_COUNT=$(find . -type f -name "*.pyo" 2>/dev/null | wc -l | tr -d ' ')
PYTEST_COUNT=$(find . -type d -name ".pytest_cache" 2>/dev/null | wc -l | tr -d ' ')

echo "Found:"
echo "  __pycache__ directories: $PYCACHE_COUNT"
echo "  .pyc files: $PYC_COUNT"
echo "  .pyo files: $PYO_COUNT"
echo "  .pytest_cache directories: $PYTEST_COUNT"
echo ""

# Remove __pycache__ directories
if [ "$PYCACHE_COUNT" -gt 0 ]; then
    echo "Removing __pycache__ directories..."
    find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
fi

# Remove .pyc files
if [ "$PYC_COUNT" -gt 0 ]; then
    echo "Removing .pyc files..."
    find . -type f -name "*.pyc" -delete 2>/dev/null || true
fi

# Remove .pyo files
if [ "$PYO_COUNT" -gt 0 ]; then
    echo "Removing .pyo files..."
    find . -type f -name "*.pyo" -delete 2>/dev/null || true
fi

# Remove .pytest_cache directories
if [ "$PYTEST_COUNT" -gt 0 ]; then
    echo "Removing .pytest_cache directories..."
    find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true
fi

# Remove .pyc files in root .pytest_cache if it exists
if [ -d ".pytest_cache" ]; then
    echo "Removing root .pytest_cache directory..."
    rm -rf .pytest_cache
fi

# Remove any lingering .coverage files
if [ -f ".coverage" ]; then
    echo "Removing .coverage file..."
    rm -f .coverage
fi

echo ""
echo "✓ Python cache cleanup completed!"