#!/usr/bin/env bash
# SessionStart hook — load the prompt-engineering method once per session.
#
# Injects the full contents of ~/Documents/promptengineering.md plus a standing
# directive so the model, for the whole session, rewrites each non-trivial
# prompt into a clearer version and shows it at the top of the response.
#
# The per-prompt UserPromptSubmit hook (engineer_prompt.sh) then only needs to
# post a short reminder, since the full method already lives in session context.
#
# Configured in .claude/settings.local.json under hooks.SessionStart.

set -euo pipefail

# Consume stdin (Claude Code passes session JSON) so the pipe never blocks.
cat >/dev/null 2>&1 || true

REF="${HOME}/Documents/promptengineering.md"

if [[ -f "$REF" ]]; then
  METHOD="$(cat "$REF")"
else
  METHOD="(Reference file not found at ${REF}. Apply general prompt-engineering best practice.)"
fi

read -r -d '' HEADER <<EOF || true
<prompt_engineering_session_directive>
Standing directive for this entire session: on every NON-TRIVIAL prompt Ra
sends, first rewrite it into a single, clearer prompt — same intent and scope,
but with more accurate, descriptive, and unambiguous wording, filling in
obviously-implied specifics without inventing new scope. Show that rewritten
prompt at the very top of your response under a short "**Engineered prompt**"
heading (a blockquote is fine), then act on the rewritten version as the real
request. Skip the rewrite for trivial messages (greetings, a plain yes/no, an
acknowledgement, or a direct command needing no interpretation).

This never overrides project rules in CLAUDE.md (e.g. never command the live arm
without Ra's explicit go-ahead; no commits without the trigger phrase).

The full prompt-engineering method Ra follows is included below for reference.
</prompt_engineering_session_directive>
EOF

CONTEXT="${HEADER}

===== ~/Documents/promptengineering.md =====
${METHOD}"

if command -v jq >/dev/null 2>&1; then
  jq -n --arg ctx "$CONTEXT" \
    '{hookSpecificOutput: {hookEventName: "SessionStart", additionalContext: $ctx}}'
else
  CONTEXT="$CONTEXT" python3 -c 'import json,os; print(json.dumps({"hookSpecificOutput":{"hookEventName":"SessionStart","additionalContext":os.environ["CONTEXT"]}}))'
fi
