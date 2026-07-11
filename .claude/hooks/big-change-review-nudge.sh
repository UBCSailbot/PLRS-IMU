#!/usr/bin/env bash
# Stop hook: when the branch has accumulated a large change versus its upstream,
# nudge the model (once per size tier) to OFFER an adversarial code review.
# Non-blocking: it only injects context. Wired from .claude/settings.json.
set -u

cd "${CLAUDE_PROJECT_DIR:-.}" 2>/dev/null || exit 0
git rev-parse --is-inside-work-tree >/dev/null 2>&1 || exit 0

# Baseline: the branch's upstream. Diffing the working tree against it captures
# unpushed commits plus uncommitted edits, i.e. everything not yet reviewed.
base=$(git rev-parse --abbrev-ref --symbolic-full-name '@{u}' 2>/dev/null) || exit 0
[ -n "$base" ] || exit 0
git rev-parse --verify -q "$base" >/dev/null 2>&1 || exit 0

read -r files lines <<EOF
$(git diff --numstat "$base" -- 2>/dev/null | awk '
  { f++; a = ($1 == "-" ? 0 : $1); d = ($2 == "-" ? 0 : $2); l += a + d }
  END { printf "%d %d", f + 0, l + 0 }')
EOF

THRESH_FILES=8
THRESH_LINES=400
tier=$(( files / THRESH_FILES ))
tl=$(( lines / THRESH_LINES ))
[ "$tl" -gt "$tier" ] && tier=$tl

# Dedupe per size tier: nudge only when a higher tier is first reached. Storing
# the current tier every run means a shrink (after a push/merge) resets it, so a
# later big change nudges again.
state="$(git rev-parse --git-dir)/claude-review-nudge-tier"
stored=$(cat "$state" 2>/dev/null || echo 0)
printf '%s' "$tier" >"$state" 2>/dev/null || true

[ "$tier" -ge 1 ] && [ "$tier" -gt "$stored" ] || exit 0

msg="A large change has accumulated on this branch: ${files} files and ${lines} lines versus ${base}. Per the user's standing preference, proactively offer to launch an adversarial code-review agent (an Opus review subagent) before wrapping up, and launch it only if the user agrees."
printf '{"hookSpecificOutput":{"hookEventName":"Stop","additionalContext":"%s"}}\n' "$msg"
