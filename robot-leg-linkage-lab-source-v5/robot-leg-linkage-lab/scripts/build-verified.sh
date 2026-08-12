#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${SITES_ENV_READY:-}" != "1" ]]; then
  exec "${script_dir}/sites-env.sh" -- "$0" "$@"
fi

command -v timeout || {
  echo "build-verified.sh requires GNU timeout." >&2
  exit 69
}

vinext="${SITES_PROJECT_ROOT}/node_modules/.bin/vinext"
if [[ ! -x "${vinext}" ]]; then
  echo "vinext is unavailable. Run npm run install:ci and wait for it to finish before building." >&2
  exit 69
fi

echo "Running bounded vinext build..."
timeout_args=(
  --signal=TERM
  --kill-after="${SITES_BUILD_KILL_AFTER:-10s}"
  "${SITES_BUILD_TIMEOUT:-3m}"
)

if command -v node >/dev/null 2>&1 && [[ "$(command -v node)" != *".sites-runtime/bin/node" ]]; then
  timeout "${timeout_args[@]}" "${vinext}" build
elif command -v node.exe >/dev/null 2>&1 && command -v wslpath >/dev/null 2>&1; then
  # npm on Windows launches this Bash script in WSL, where package shims cannot
  # find a POSIX `node`. Run the same vinext CLI through the available Windows
  # executable with a Windows path instead.
  timeout "${timeout_args[@]}" node.exe "$(wslpath -w "${SITES_PROJECT_ROOT}/node_modules/vinext/dist/cli.js")" build
else
  echo "Node.js is unavailable to the verified build." >&2
  exit 69
fi

"${script_dir}/validate-artifact.sh"
