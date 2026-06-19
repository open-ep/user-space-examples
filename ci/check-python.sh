#!/usr/bin/env bash
set -euo pipefail

files=()
while IFS= read -r -d '' file; do
    files+=("$file")
done < <(find . -path ./.git -prune -o -name '*.py' -print0)

if [ "${#files[@]}" -eq 0 ]; then
    echo "No Python files found."
    exit 0
fi

echo "Checking Python syntax for ${#files[@]} file(s)."
export PYTHONPYCACHEPREFIX="${PYTHONPYCACHEPREFIX:-${TMPDIR:-/tmp}/user-space-examples-pycache}"
python3 -m py_compile "${files[@]}"
