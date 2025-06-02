import os
import pytest

@pytest.mark.parametrize("filename", [
    "test_flake8.py",
    "test_pep257.py",
    "test_copyright.py"
])
def test_lint_files_exist(filename):
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    test_file = os.path.join(pkg_dir, 'test', filename)
    assert os.path.exists(test_file), f"Lint test file not found: {test_file}"
