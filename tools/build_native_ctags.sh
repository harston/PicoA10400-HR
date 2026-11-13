#!/usr/bin/env bash
# Build arduino's ctags (tag 5.8-arduino11 - the exact version arduino-cli
# installs as builtin:ctags) as a native binary into tools/.cache/ctags/.
#
# WHY. arduino-cli runs ctags to generate the sketch's function prototypes.
# The copy it downloads exists only for x86_64, and macOS 27 no longer ships
# Rosetta 2, so every compile stopped with
#     fork/exec .../builtin/tools/ctags/5.8-arduino11/ctags: bad CPU type in executable
# build.sh passes --build-property tools.ctags.path=tools/.cache/ctags when
# the binary built here exists; nothing in the Arduino installation is touched.
#
# The one source change: general.h defines a macro named __unused__, which the
# current macOS SDK's <dirent.h> collides with (it uses __unused, defined as
# __attribute__((__unused__))). Renaming ctags' macro is what distributions do
# for this 2009 code base; it only affects an "unused variable" attribute.
#
# Proof that the result is equivalent: the 0.73 tree compiled with this ctags
# gave PicoA10400.ino.uf2 01a65695e4f489dbda6a27f0a56b5414 and
# Pico2A10400.ino.uf2 9fafeacb56432c20b38c6b3fbb6dff02 - byte-identical to the
# images shipped with 0.73.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$HERE/.cache/ctags"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

git clone -q --depth 1 --branch 5.8-arduino11 https://github.com/arduino/ctags.git "$WORK/src"
cd "$WORK/src"
for f in $(grep -rl "__unused__" -- *.c *.h); do
  perl -pi -e 's/\b__unused__\b/CTAGS_UNUSED_ATTR/g' "$f"
done
./configure --disable-external-sort \
  CFLAGS="-O2 -Wno-implicit-function-declaration -Wno-int-conversion -Wno-deprecated-non-prototype" >/dev/null
make -j8 >/dev/null
mkdir -p "$DEST"
cp ctags "$DEST/ctags"
file "$DEST/ctags"
"$DEST/ctags" --version | head -1
