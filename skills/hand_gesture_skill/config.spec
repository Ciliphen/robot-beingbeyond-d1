# Runtime config accepted by the hand_gesture skill.
#
# This file documents the mapping passed as this package's `config:` value in
# robonix_manifest.yaml. The hand_gesture skill has NO tunable config — the
# dance duration is a per-call MCP argument, and the wiggle amplitude / step
# interval / start pose are fixed to the SDK gesture_dance defaults. Pass an
# empty `config: {}`.

config: {}
