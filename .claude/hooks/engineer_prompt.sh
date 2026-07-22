#!/usr/bin/env bash
# UserPromptSubmit hook — "prompt engineering" pass.
#
# On every prompt Ra submits, this injects a directive telling the answering
# model to first re-express the request as an engineered prompt (using the
# method recorded in ~/Documents/promptengineering.md), show it, and then act
# on the engineered version instead of the raw wording.
#
# It does NOT call a second model — the same Claude does the rewrite inline,
# so there is no extra latency or token cost beyond this small directive.
#
# Configured in .claude/settings.local.json under hooks.UserPromptSubmit.

set -euo pipefail

# Consume stdin (Claude Code passes the prompt as JSON) so the pipe never blocks.
cat >/dev/null 2>&1 || true

REF="${HOME}/Documents/promptengineering.md"

read -r -d '' CONTEXT <<EOF || true
<prompt_engineering_reminder>
Per the session prompt-engineering directive: rewrite this prompt into a single
clearer, more accurate and descriptive version (same intent and scope), show it
at the top of your response under a "**Engineered prompt**" heading, then act on
the rewritten version. Skip for trivial messages (greetings, yes/no,
acknowledgements, or direct commands needing no interpretation).
</prompt_engineering_reminder>
EOF

# Emit as additionalContext. Prefer jq for safe JSON encoding; fall back to
# python3 (always present in this project) if jq is unavailable.
if command -v jq >/dev/null 2>&1; then
  jq -n --arg ctx "$CONTEXT" \
    '{hookSpecificOutput: {hookEventName: "UserPromptSubmit", additionalContext: $ctx}}'
else
  CONTEXT="$CONTEXT" python3 -c 'import json,os; print(json.dumps({"hookSpecificOutput":{"hookEventName":"UserPromptSubmit","additionalContext":os.environ["CONTEXT"]}}))'
fi
