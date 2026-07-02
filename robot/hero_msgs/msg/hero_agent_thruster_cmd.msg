# RL thruster mixer -> firmware ESC command (2026-07-02).
# 6 per-thruster commands in [-1, 1], channel order T0..T5 = firmware m0..m5.
# The mixer (thruster_mixer node) publishes this after applying the measured
# per-channel sign table; agent.ino's sub_thruster callback maps each channel
# to an ESC PWM (NEUTRAL 1500 +- span) and drives esc_input over UART2.
# Sign/scale conventions and the PWM mapping live in the mixer + firmware, NOT here.
float32[6] thrust
