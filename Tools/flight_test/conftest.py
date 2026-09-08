import os
import sys

# test_card_report.py is the CLI module, not a test module.
collect_ignore = ["test_card_report.py"]

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
