"""extractPy turns raw LLM responses into runnable .py: it must pull code out of markdown
fences, tolerate a fence-free response, and strip comments for the cleaned variant."""
from extractPy import extract_python_code, remove_comments


def test_extract_from_python_fence(tmp_path):
    txt = tmp_path / "first_response.txt"
    txt.write_text("Here is the script:\n```python\nx = 1\nprint(x)\n```\nHope it helps.\n",
                   encoding="utf-8")
    out = tmp_path / "first_response.py"
    extract_python_code(str(txt), str(out), log_file=str(tmp_path / "extract.log"))
    code = out.read_text(encoding="utf-8")
    assert "x = 1" in code and "print(x)" in code
    assert "Here is the script" not in code and "Hope it helps" not in code


def test_extract_without_fence_keeps_whole_body(tmp_path):
    txt = tmp_path / "first_response.txt"
    txt.write_text("import pychrono as chrono\nsys = chrono.ChSystemNSC()\n", encoding="utf-8")
    out = tmp_path / "first_response.py"
    extract_python_code(str(txt), str(out), log_file=str(tmp_path / "extract.log"))
    code = out.read_text(encoding="utf-8")
    assert "import pychrono as chrono" in code


def test_remove_comments_strips_comments_and_docstrings():
    src = 'x = 1  # set x\n"""module doc"""\ny = 2\n'
    cleaned = remove_comments(src)
    assert "x = 1" in cleaned and "y = 2" in cleaned
    assert "set x" not in cleaned and "module doc" not in cleaned
