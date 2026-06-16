#!/usr/bin/env bash
# Package the built FMI 2.0 Co-Simulation slave into a standards-compliant .fmu archive.
#
# An .fmu is a plain zip with a fixed layout (FMI 2.0 §2.1.5):
#   modelDescription.xml          (at the archive root)
#   binaries/<platform>/<id>.so   (the shared library; <id> == modelIdentifier == "gncsim")
#
# Requires a build configured with -DGNCSIM_BUILD_FMI=ON (so gncsim.so + modelDescription.xml
# exist in the fmi build dir). Usage:
#   fmi/scripts/package_fmu.sh <build-dir> [out.fmu]
# e.g.  fmi/scripts/package_fmu.sh build-fmi build-fmi/gncsim.fmu
set -euo pipefail

BUILD_DIR="${1:-build-fmi}"
OUT_FMU="${2:-${BUILD_DIR}/gncsim.fmu}"

# The slave .so and the generated descriptor land in the fmi/ subtree of the build dir.
FMI_BIN_DIR="${BUILD_DIR}/fmi"
SO_PATH="${FMI_BIN_DIR}/gncsim.so"
XML_PATH="${FMI_BIN_DIR}/modelDescription.xml"

if [[ ! -f "${SO_PATH}" ]]; then
  echo "package_fmu: missing ${SO_PATH} — build with -DGNCSIM_BUILD_FMI=ON first" >&2
  exit 1
fi
if [[ ! -f "${XML_PATH}" ]]; then
  echo "package_fmu: missing ${XML_PATH} — build target gncsim_fmi_modeldesc first" >&2
  exit 1
fi

# FMI 2.0 platform directory name for 64-bit Linux.
PLATFORM="linux64"

STAGE="$(mktemp -d)"
trap 'rm -rf "${STAGE}"' EXIT
mkdir -p "${STAGE}/binaries/${PLATFORM}"
cp "${XML_PATH}" "${STAGE}/modelDescription.xml"
cp "${SO_PATH}" "${STAGE}/binaries/${PLATFORM}/gncsim.so"

OUT_FMU_ABS="$(cd "$(dirname "${OUT_FMU}")" && pwd)/$(basename "${OUT_FMU}")"
rm -f "${OUT_FMU_ABS}"
( cd "${STAGE}" && zip -r -q "${OUT_FMU_ABS}" . )

echo "package_fmu: wrote ${OUT_FMU_ABS}"
echo "package_fmu: contents:"
unzip -l "${OUT_FMU_ABS}"
