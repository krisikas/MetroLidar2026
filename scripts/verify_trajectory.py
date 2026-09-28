#!/usr/bin/env python3
"""
Offline mathematical verification of the MetroLidar trajectory & obstacle detection pipeline
running across all hackathon datasets and synthetic obstacle frames.
"""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from scripts.verify_system import run_verification

if __name__ == '__main__':
    run_verification()
