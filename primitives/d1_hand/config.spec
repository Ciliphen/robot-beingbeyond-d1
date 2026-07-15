# Runtime config accepted by the D1 dexterous-hand primitive.
#
# This file documents the mapping passed as this package's `config:` value in
# robonix_manifest.yaml. It is documentation for deployers and tooling; the
# provider continues to parse and validate the values in its own code
# (each key also falls back to the matching D1_HAND_* env var).

config:
  # string, right | left, default: right (env: D1_HAND_TYPE).
  # Which physical hand this instance drives. Selects the axis sign / mapping
  # inside the DexHand SDK.
  hand_type: right

  # string, default: can0 (env: D1_HAND_CAN).
  # CAN network interface the hand is wired to. DexHand brings this interface
  # up at init (`ip link set …`); if not run as root, export PREFLIGHT_SUDO_PASS
  # or bring the interface up beforehand.
  can_iface: can0

  # int, default: 1000000 (env: D1_HAND_BAUD).
  # CAN bitrate for the hand link. Must match the hand controller firmware.
  baudrate: 1000000
