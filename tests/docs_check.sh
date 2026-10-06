#!/bin/sh
# Checks the hand-written docs cover the code:
#  1. every header include/pt/<cat>/<name>.h has an element id="<cat>-<name>"
#  2. every inline function in include/pt/core/<file>.h has an element
#     id="core-<file>-<function>"
#  3. every documented section (an <h2>/<h3> with an id) contains at least
#     one <pre> code example before the next heading
# Prints what is missing and exits non-zero if anything is.

cd "$(dirname "$0")/.." || exit 1
missing=0
docs=docs/*.html

for h in $(cd include/pt && find . -name '*.h' -path './*/*' | sed 's|^\./||;s|\.h$||;s|/|-|g'); do
  grep -q "id=\"$h\"" $docs || { echo "docs: no section for header $h"; missing=1; }
done

for f in include/pt/core/*.h; do
  base=$(basename "$f" .h)
  for fn in $(sed -n 's/^inline [^(]* \**\([A-Za-z0-9_]*\)(.*/\1/p' "$f"); do
    grep -q "id=\"core-$base-$fn\"" $docs || { echo "docs: no section for core function $base/$fn"; missing=1; }
  done
done

for d in $docs; do
  awk -v file="$d" '
    /<h[23][^>]* id="/ {
      if (id != "" && !hasPre) { print "docs: " file ": section " id " has no code example"; bad = 1 }
      match($0, /id="[^"]*"/); id = substr($0, RSTART + 4, RLENGTH - 5); hasPre = 0; next
    }
    # An <h2> without an id ends a section; an <h3> without one is a
    # sub-heading inside it.
    /<h2/ { if (id != "" && !hasPre) { print "docs: " file ": section " id " has no code example"; bad = 1 } id = "" }
    /<pre>/ { hasPre = 1 }
    END { if (id != "" && !hasPre) { print "docs: " file ": section " id " has no code example"; bad = 1 } exit bad }
  ' "$d" || missing=1
done

[ $missing = 0 ] && echo "docs: ok"
exit $missing
