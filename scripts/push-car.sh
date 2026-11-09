#!/usr/bin/env bash
# Option B: push the car/ folder back to the original PeaceOfMine repository.
# Run from a local clone of THIS project's GitHub repo:
#   scripts/push-car.sh [peaceofmine-remote-url] [branch]
set -euo pipefail
REMOTE="${1:-https://github.com/HarjodhsGitHub/PeaceOfMine.git}"
BRANCH="${2:-lovable-dashboard}"
git subtree push --prefix car "$REMOTE" "$BRANCH"
echo "Pushed car/ to $REMOTE ($BRANCH). Open a pull request there to merge it."
